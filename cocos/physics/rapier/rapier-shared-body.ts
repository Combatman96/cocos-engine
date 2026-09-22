/*
 Copyright (c) 2020-2023 Xiamen Yaji Software Co., Ltd.

 https://www.cocos.com/

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Quat, Vec3 } from '../../core';
import { Node } from '../../scene-graph';
import { TransformBit } from '../../scene-graph/node-enum';
import { PhysicsSystem } from '../../../exports/physics-framework';
import { ERigidBodyType, PhysicsGroup } from '../framework/physics-enum';
import { RapierWorld } from './rapier-world';
import { RapierCache, CC_QUAT_0, CC_V3_0 } from './rapier-cache';
import { ERapierSharedBodyDirty, toRapierBodyType } from './rapier-enum';
import { checkGroupWidth, packInteractionGroups } from './rapier-utils';
import { R } from './instantiated';
import type { RapierShape } from './shapes/rapier-shape';
import type { RapierRigidBody } from './rapier-rigid-body';

/**
 * node : shared-body = 1 : 1
 *
 * One Rapier rigid body per Node, shared by that node's `RigidBody` component (at most
 * one) and all of its `Collider` components. This mirrors the arrangement every other
 * Cocos physics backend uses.
 *
 * Simpler than the bullet equivalent in one important way: Rapier sensors live on the
 * same rigid body as solid colliders, so there is no separate ghost object and no
 * move-between-compounds dance when `isTrigger` is toggled — it is just `setSensor`.
 */
export class RapierSharedBody {
    private static readonly sharedBodesMap = new Map<string, RapierSharedBody>();

    static getSharedBody (node: Node, wrappedWorld: RapierWorld, wrappedBody?: RapierRigidBody): RapierSharedBody {
        const key = node.uuid;
        let newSB: RapierSharedBody;
        if (RapierSharedBody.sharedBodesMap.has(key)) {
            newSB = RapierSharedBody.sharedBodesMap.get(key)!;
        } else {
            newSB = new RapierSharedBody(node, wrappedWorld);
            const g = PhysicsGroup.DEFAULT;
            newSB._collisionFilterGroup = g;
            newSB._collisionFilterMask = PhysicsSystem.instance.collisionMatrix[g];
            RapierSharedBody.sharedBodesMap.set(key, newSB);
        }
        if (wrappedBody) {
            newSB._wrappedBody = wrappedBody;
            const g = wrappedBody.rigidBody.group;
            newSB._collisionFilterGroup = g;
            newSB._collisionFilterMask = PhysicsSystem.instance.collisionMatrix[g];
        }
        return newSB;
    }

    readonly node: Node;
    readonly wrappedWorld: RapierWorld;
    readonly wrappedShapes: RapierShape[] = [];

    dirty: ERapierSharedBodyDirty = 0;

    private _body: RAPIER.RigidBody | null = null;
    private _wrappedBody: RapierRigidBody | null = null;
    private _collisionFilterGroup: number = PhysicsGroup.DEFAULT;
    private _collisionFilterMask = -1;
    private _isKinematic = false;
    private _allowSleep = true;
    private _mass = 1;
    private _index = -1;
    private _ref = 0;

    /** Set while the body has pending force/torque that must be cleared after the step. */
    private _forcesDirty = false;

    private constructor (node: Node, wrappedWorld: RapierWorld) {
        this.node = node;
        this.wrappedWorld = wrappedWorld;
    }

    get wrappedBody (): RapierRigidBody | null {
        return this._wrappedBody;
    }

    get isKinematic (): boolean {
        return this._isKinematic;
    }

    get allowSleep (): boolean {
        return this._allowSleep;
    }

    get index (): number {
        return this._index;
    }

    set index (v: number) {
        this._index = v;
    }

    get collisionFilterGroup (): number {
        return this._collisionFilterGroup;
    }

    set collisionFilterGroup (v: number) {
        if (v !== this._collisionFilterGroup) {
            checkGroupWidth(v);
            this._collisionFilterGroup = v;
            this._updateFilterData();
        }
    }

    get collisionFilterMask (): number {
        return this._collisionFilterMask;
    }

    set collisionFilterMask (v: number) {
        if (v !== this._collisionFilterMask) {
            checkGroupWidth(v);
            this._collisionFilterMask = v;
            this._updateFilterData();
        }
    }

    /**
     * The Rapier body, created on first access.
     *
     * Creation is deferred because at `Collider.onLoad` time we may not yet know whether
     * the node also carries a `RigidBody` component — a node with colliders but no rigid
     * body is a static body in Cocos.
     */
    get impl (): RAPIER.RigidBody {
        if (!this._body) {
            // Only an *enabled* RigidBody component gets to dictate the body type. A node
            // with colliders but no (or a disabled) rigid body is static in Cocos, and
            // `RapierRigidBody.onEnable` calls `setType` afterwards to correct it —
            // which works in either component-order because it sets `isEnabled` first.
            const driving = this._wrappedBody && this._wrappedBody.isEnabled ? this._wrappedBody.rigidBody : null;
            const type = driving ? toRapierBodyType(driving.type) : toRapierBodyType(ERigidBodyType.STATIC);
            this._isKinematic = !!driving && driving.type === ERigidBodyType.KINEMATIC;
            const wp = this.node.worldPosition;
            const desc = new R.RigidBodyDesc(type as RAPIER.RigidBodyType)
                .setTranslation(wp.x, wp.y, wp.z)
                .setRotation(this.node.worldRotation)
                .setCanSleep(PhysicsSystem.instance.allowSleep);
            desc.userData = this;
            this._body = this.wrappedWorld.impl.createRigidBody(desc);
            RapierCache.setBody(this._body.handle, this);
        }
        return this._body;
    }

    /** True once the Rapier body exists, without forcing it into existence. */
    get hasBody (): boolean {
        return this._body !== null;
    }

    setWrappedBody (v: RapierRigidBody): void {
        this._wrappedBody = v;
    }

    /**
     * Add to / remove from the world.
     * Add when enabled; remove only when there is nothing left referencing the body.
     *
     * Rapier can take a body out of the broad phase with `setEnabled` without
     * invalidating handles, so unlike cannon and bullet we never remove and reinsert.
     */
    set enabled (v: boolean) {
        if (v) {
            if (this._index < 0) {
                this._index = this.wrappedWorld.bodies.length;
                this.wrappedWorld.addSharedBody(this);
                this.syncInitial();
                this.impl.setEnabled(true);
            }
        } else if (this._index >= 0) {
            const isRemove = (this.wrappedShapes.length === 0 && this._wrappedBody === null)
                || (this.wrappedShapes.length === 0 && this._wrappedBody !== null && !this._wrappedBody.isEnabled);
            if (isRemove) {
                this.clearVelocity();
                if (this._body) this._body.setEnabled(false);
                this._index = -1;
                this.wrappedWorld.removeSharedBody(this);
            }
        }
    }

    set reference (v: boolean) {
        // eslint-disable-next-line @typescript-eslint/no-unused-expressions
        v ? this._ref++ : this._ref--;
        if (this._ref === 0) this.destroy();
    }

    addShape (v: RapierShape): void {
        const index = this.wrappedShapes.indexOf(v);
        if (index < 0) {
            this.wrappedShapes.push(v);
            v.createCollider(this.impl);
            this.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
        }
    }

    removeShape (v: RapierShape): void {
        const index = this.wrappedShapes.indexOf(v);
        if (index >= 0) {
            this.wrappedShapes.splice(index, 1);
            v.destroyCollider();
            this.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
        }
    }

    setType (v: ERigidBodyType): void {
        this._isKinematic = (v === ERigidBodyType.KINEMATIC);
        this.impl.setBodyType(toRapierBodyType(v) as RAPIER.RigidBodyType, true);
        if (v === ERigidBodyType.DYNAMIC) {
            const com = this._wrappedBody?.rigidBody;
            if (com) {
                this.setMass(com.mass);
                this.impl.setGravityScale(com.useGravity ? 1 : 0, true);
                this.setAllowSleep(com.allowSleep);
            }
        } else {
            this.clearVelocity();
        }
    }

    setMass (v: number): void {
        this._mass = v;
        this.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    get mass (): number {
        return this._mass;
    }

    setAllowSleep (v: boolean): void {
        this._allowSleep = v;
        // Rapier exposes `setCanSleep` on RigidBodyDesc only; there is no runtime setter.
        // When sleeping is disallowed we force-wake instead (see `forceWakeIfNeeded`).
        if (!v && this._body && this._body.isSleeping()) this._body.wakeUp();
    }

    clearVelocity (): void {
        if (!this._body) return;
        CC_V3_0.set(0, 0, 0);
        this._body.setLinvel(CC_V3_0, false);
        this._body.setAngvel(CC_V3_0, false);
    }

    clearForces (): void {
        if (!this._body) return;
        this._body.resetForces(false);
        this._body.resetTorques(false);
        this._forcesDirty = false;
    }

    /** Marks that a force or torque was accumulated this frame. */
    markForcesDirty (): void {
        this._forcesDirty = true;
    }

    /**
     * Cocos treats `applyForce`/`applyTorque` as a single-step impulse of force, matching
     * bullet and cannon which clear the accumulator after every step. Rapier's
     * `addForce`/`addTorque` persist until explicitly reset, so the world clears them at
     * the tail of each step for any body that accumulated some.
     */
    resetForcesIfDirty (): void {
        if (this._forcesDirty && this._body) {
            this._body.resetForces(false);
            this._body.resetTorques(false);
            this._forcesDirty = false;
        }
    }

    /** Re-wakes a body whose component forbids sleeping. */
    forceWakeIfNeeded (): void {
        if (!this._allowSleep && this._body && this._body.isDynamic() && this._body.isSleeping()) {
            this._body.wakeUp();
        }
    }

    /** Flushes deferred work; called once per `syncSceneToPhysics`. */
    updateDirty (): void {
        if (this.dirty & ERapierSharedBodyDirty.MASS_PROPERTIES) {
            this._flushMassProperties();
        }
        this.dirty = 0;
    }

    syncSceneToPhysics (): void {
        this.updateDirty();
        const node = this.node;
        if (node.hasChangedFlags) {
            if (node.hasChangedFlags & TransformBit.SCALE) this.syncScale();
            const body = this.impl;
            if (this._isKinematic) {
                // Driving a kinematic body through `setNextKinematic*` rather than
                // `setTranslation` is what gives it contact velocity, so dynamic bodies
                // resting on a moving kinematic platform get pushed along.
                body.setNextKinematicTranslation(node.worldPosition);
                body.setNextKinematicRotation(node.worldRotation);
            } else {
                body.setTranslation(node.worldPosition, true);
                body.setRotation(node.worldRotation, true);
            }
        }
        this.forceWakeIfNeeded();
    }

    /** Writes the simulated transform back onto the node. Called after the step. */
    syncPhysicsToScene (): void {
        const body = this._body;
        if (!body || body.isSleeping()) return;
        this.node.worldPosition = body.translation(CC_V3_0) as Vec3;
        this.node.worldRotation = body.rotation(CC_QUAT_0) as Quat;
    }

    /**
     * Like `syncSceneToPhysics` but skips the write when the node's transform already
     * matches the body. Used by `syncAfterEvents`, because the post-step writeback set
     * `hasChangedFlags` itself — an unconditional resync there would push the body's own
     * output back into the solver and discard its predicted position.
     */
    syncSceneWithCheck (): void {
        const node = this.node;
        if (!node.hasChangedFlags) return;
        const body = this.impl;
        const samePos = Vec3.equals(body.translation(CC_V3_0) as Vec3, node.worldPosition);
        const sameRot = Quat.equals(body.rotation(CC_QUAT_0) as Quat, node.worldRotation);
        if (samePos && sameRot) return;
        if (node.hasChangedFlags & TransformBit.SCALE) this.syncScale();
        if (this._isKinematic) {
            body.setNextKinematicTranslation(node.worldPosition);
            body.setNextKinematicRotation(node.worldRotation);
        } else {
            body.setTranslation(node.worldPosition, true);
            body.setRotation(node.worldRotation, true);
        }
    }

    syncInitial (): void {
        const body = this.impl;
        if (this._isKinematic) {
            body.setNextKinematicTranslation(this.node.worldPosition);
            body.setNextKinematicRotation(this.node.worldRotation);
        } else {
            body.setTranslation(this.node.worldPosition, false);
            body.setRotation(this.node.worldRotation, false);
        }
        this.clearVelocity();
        this.syncScale();
    }

    syncScale (): void {
        for (let i = 0; i < this.wrappedShapes.length; i++) {
            this.wrappedShapes[i].syncScale();
        }
        this.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    destroy (): void {
        // Handles are recycled arena indices, so every registry entry must be dropped
        // before anything else can claim the same handle.
        for (let i = 0; i < this.wrappedShapes.length; i++) {
            this.wrappedShapes[i].destroyCollider();
        }
        this.wrappedShapes.length = 0;
        if (this._body) {
            RapierCache.delBody(this._body.handle);
            this.wrappedWorld.impl.removeRigidBody(this._body);
            this._body = null;
        }
        RapierSharedBody.sharedBodesMap.delete(this.node.uuid);
        if (this._index >= 0) {
            this._index = -1;
            this.wrappedWorld.removeSharedBody(this);
        }
        this._wrappedBody = null;
        (this.node as unknown) = null;
    }

    private _updateFilterData (): void {
        const packed = packInteractionGroups(this._collisionFilterGroup, this._collisionFilterMask);
        for (let i = 0; i < this.wrappedShapes.length; i++) {
            const impl = this.wrappedShapes[i].impl;
            if (impl) impl.setCollisionGroups(packed);
        }
    }

    /**
     * Distributes the component's scalar mass across the body's solid colliders.
     *
     * Cocos models mass as one number on the body; Rapier derives it from each collider's
     * density or explicit mass. We set explicit per-collider masses in proportion to
     * volume, which keeps the total equal to `rigidBody.mass` and yields a correct inertia
     * tensor for the common single-shape case.
     *
     * Triggers are excluded because Cocos treats them as massless.
     */
    private _flushMassProperties (): void {
        const body = this._body;
        if (!body || !body.isDynamic()) return;

        const m = Math.max(this._mass, 1e-6);
        const solids: RapierShape[] = [];
        for (let i = 0; i < this.wrappedShapes.length; i++) {
            const s = this.wrappedShapes[i];
            if (!s.isTrigger && s.isEnabled && s.impl) solids.push(s);
        }

        if (solids.length === 0) {
            // A dynamic body with no solid collider has zero mass, and Rapier freezes
            // zero-mass dynamic bodies outright. An explicit additional mass keeps it
            // well-defined (inertia is degenerate, but the body still falls).
            body.setAdditionalMass(m, true);
        } else if (solids.length === 1) {
            body.setAdditionalMass(0, false);
            solids[0].impl!.setMass(m);
        } else {
            let total = 0;
            for (let i = 0; i < solids.length; i++) total += solids[i].impl!.volume();
            if (total > 1e-9) {
                body.setAdditionalMass(0, false);
                for (let i = 0; i < solids.length; i++) {
                    const s = solids[i].impl!;
                    s.setMass(m * s.volume() / total);
                }
            } else {
                // Trimesh and half-space colliders have zero volume and zero mass
                // properties in Rapier, so fall back to a point mass on the body.
                for (let i = 0; i < solids.length; i++) solids[i].impl!.setDensity(0);
                body.setAdditionalMass(m, true);
            }
        }
        body.recomputeMassPropertiesFromColliders();
    }
}
