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
import { IVec3Like, Vec3, toRadian } from '../../../core';
import { TransformBit } from '../../../scene-graph/node-enum';
import { CharacterController, PhysicsSystem } from '../../../../exports/physics-framework';
import { IBaseCharacterController } from '../../spec/i-character-controller';
import { PhysicsGroup } from '../../framework/physics-enum';
import { CharacterControllerContact } from '../../framework/physics-interface';
import { RapierCache } from '../rapier-cache';
import { RapierWorld } from '../rapier-world';
import { ERapierBodyType } from '../rapier-enum';
import { packInteractionGroups } from '../rapier-utils';
import { R } from '../instantiated';

const v3_0 = new Vec3();
const v3_1 = new Vec3();
const v3_2 = new Vec3();

/**
 * Rapier's `KinematicCharacterController` sweeps a **collider**, not a rigid body, and it
 * never applies gravity of its own.
 *
 * The wrapper therefore owns the character's world position: it keeps a kinematic body
 * carrying the shape so other bodies can collide with it, asks the controller how far a
 * requested move may actually go, and writes the result back onto the node.
 */
/** @mangle */
export abstract class RapierCharacterController implements IBaseCharacterController {
    protected _comp!: CharacterController;
    protected _world!: RapierWorld;
    protected _controller: RAPIER.KinematicCharacterController | null = null;
    protected _body: RAPIER.RigidBody | null = null;
    protected _collider: RAPIER.Collider | null = null;

    private readonly _position = new Vec3();
    private _grounded = false;
    private _isEnabled = false;
    private _destroyed = false;
    private _collisionFilterGroup: number = PhysicsGroup.DEFAULT;
    private _collisionFilterMask = -1;

    /** Contacts produced by the most recent sweep, drained by the world after the step. */
    readonly pendingContacts: CharacterControllerContact[] = [];
    private readonly _contactPool: CharacterControllerContact[] = [];
    private _collisionScratch: RAPIER.CharacterCollision | null = null;

    private static _idCounter = 0;
    readonly id = RapierCharacterController._idCounter++;

    get impl (): RAPIER.KinematicCharacterController | null {
        return this._controller;
    }

    get characterController (): CharacterController {
        return this._comp;
    }

    get collider (): RAPIER.Collider | null {
        return this._collider;
    }

    /** Builds the shape descriptor for the concrete controller. */
    protected abstract buildColliderDesc (): RAPIER.ColliderDesc;

    initialize (comp: CharacterController): boolean {
        this._comp = comp;
        this._world = PhysicsSystem.instance.physicsWorld as RapierWorld;
        this._collisionFilterGroup = comp.group;
        this._collisionFilterMask = PhysicsSystem.instance.collisionMatrix[comp.group];

        Vec3.add(this._position, comp.node.worldPosition, this.scaledCenter);
        this._createNative();
        return this._controller !== null;
    }

    onLoad (): void { /* nothing to do */ }

    onEnable (): void {
        this._isEnabled = true;
        if (!this._controller) this._createNative();
        this._world.addCCT(this);
        if (this._collider) this._collider.setEnabled(true);
    }

    onDisable (): void {
        this._isEnabled = false;
        this._world.removeCCT(this);
        if (this._collider) this._collider.setEnabled(false);
    }

    onDestroy (): void {
        // `RapierWorld.destroy()` tears controllers down directly, so component teardown
        // can reach here a second time - and by then the wasm world is freed.
        if (this._destroyed) return;
        this._destroyed = true;
        this._world.removeCCT(this);
        this._destroyNative();
        (this._comp as unknown) = null;
    }

    getPosition (out: IVec3Like): void {
        Vec3.copy(out, this._position);
    }

    setPosition (value: IVec3Like): void {
        Vec3.copy(this._position, value);
        this._pushPositionToNative();
        this.syncPhysicsToScene();
    }

    setStepOffset (value: number): void {
        if (!this._controller) return;
        // Autostep needs a minimum width as well as a height; a small constant keeps steps
        // usable without letting the character climb arbitrarily thin ledges.
        this._controller.enableAutostep(value, 0.01, true);
    }

    setSlopeLimit (value: number): void {
        if (!this._controller) return;
        this._controller.setMaxSlopeClimbAngle(toRadian(value));
    }

    setContactOffset (value: number): void {
        if (!this._controller) return;
        this._controller.setOffset(value);
    }

    setDetectCollisions (value: boolean): void {
        if (this._collider) this._collider.setEnabled(value);
    }

    setOverlapRecovery (_value: boolean): void {
        // Rapier resolves penetration as part of its sweep; there is no separate toggle.
    }

    onGround (): boolean {
        return this._grounded;
    }

    /**
     * Applies `movement` through the controller's sweep.
     *
     * `minDist` and `elapsedTime` arrive straight from the framework. A move shorter than
     * `minDist` is a no-op, matching every other backend. `elapsedTime` is deliberately
     * unused: the controller is a pure geometric sweep, so a zero timestep is harmless
     * rather than a division by zero.
     */
    move (movement: IVec3Like, minDist: number, _elapsedTime: number): void {
        if (!this._isEnabled || !this._controller || !this._collider) return;
        Vec3.copy(v3_0, movement);
        if (Vec3.lengthSqr(v3_0) < minDist * minDist) return;

        this._controller.computeColliderMovement(
            this._collider,
            v3_0 as RAPIER.Vector,
            undefined,
            packInteractionGroups(this._collisionFilterGroup, this._collisionFilterMask),
        );
        this._controller.computedMovement(v3_1 as RAPIER.Vector);
        this._grounded = this._controller.computedGrounded();
        Vec3.add(this._position, this._position, v3_1);
        this._pushPositionToNative();
        if (this._comp.needCollisionEvent) this._collectCollisions(v3_0);
    }

    /**
     * Copies this sweep's collisions out of Rapier.
     *
     * `computedCollision` hands back a reused view whose backing memory the next call
     * overwrites, so every field has to be copied before moving on.
     */
    private _collectCollisions (motion: Vec3): void {
        if (!this._controller) return;
        const count = this._controller.numComputedCollisions();
        const motionLength = Vec3.len(motion);
        Vec3.normalize(v3_2, motion);
        for (let i = 0; i < count; i++) {
            this._collisionScratch = this._controller.computedCollision(i, this._collisionScratch ?? undefined);
            const hit = this._collisionScratch;
            if (!hit || !hit.collider) continue;
            const shape = RapierCache.getShape(hit.collider.handle);
            if (!shape || !shape.collider) continue;

            const contact = this._contactPool.pop() ?? new CharacterControllerContact();
            contact.controller = this._comp;
            contact.collider = shape.collider;
            contact.worldPosition.set(hit.witness1.x, hit.witness1.y, hit.witness1.z);
            contact.worldNormal.set(hit.normal1.x, hit.normal1.y, hit.normal1.z);
            contact.motionDirection.set(v3_2.x, v3_2.y, v3_2.z);
            contact.motionLength = motionLength;
            this.pendingContacts.push(contact);
        }
    }

    /** Returns this frame's contacts to the pool once the world has emitted them. */
    recycleContacts (): void {
        for (let i = 0; i < this.pendingContacts.length; i++) {
            this._contactPool.push(this.pendingContacts[i]);
        }
        this.pendingContacts.length = 0;
    }

    syncPhysicsToScene (): void {
        Vec3.subtract(v3_2, this._position, this.scaledCenter);
        this._comp.node.setWorldPosition(v3_2);
    }

    /** Teleports the controller when user code moved the node. */
    syncSceneToPhysics (): void {
        const node = this._comp.node;
        if (!node.hasChangedFlags) return;
        if (node.hasChangedFlags & TransformBit.SCALE) this.updateScale();
        if (node.hasChangedFlags & TransformBit.POSITION) {
            Vec3.add(v3_0, node.worldPosition, this.scaledCenter);
            if (!Vec3.equals(v3_0, this._position)) {
                Vec3.copy(this._position, v3_0);
                this._pushPositionToNative();
            }
        }
    }

    updateEventListener (): void {
        if (this._destroyed || !this._comp || this._world.destroyed) return;
        this._world.updateNeedEmitCCTEvents(this._comp.needCollisionEvent);
    }

    /** Rebuilds the shape after a scale or dimension change. */
    updateScale (): void {
        this._rebuildCollider();
    }

    get scaledCenter (): Vec3 {
        return Vec3.multiply(v3_1, this._comp.center, this._comp.node.worldScale);
    }

    setGroup (v: number): void {
        this._collisionFilterGroup = v;
        this._applyFilter();
    }

    getGroup (): number {
        return this._collisionFilterGroup;
    }

    addGroup (v: number): void {
        this._collisionFilterGroup |= v;
        this._applyFilter();
    }

    removeGroup (v: number): void {
        this._collisionFilterGroup &= ~v;
        this._applyFilter();
    }

    setMask (v: number): void {
        this._collisionFilterMask = v;
        this._applyFilter();
    }

    getMask (): number {
        return this._collisionFilterMask;
    }

    addMask (v: number): void {
        this._collisionFilterMask |= v;
        this._applyFilter();
    }

    removeMask (v: number): void {
        this._collisionFilterMask &= ~v;
        this._applyFilter();
    }

    protected get rapier (): typeof R {
        return R;
    }

    private _createNative (): void {
        const world = this._world.impl;
        const comp = this._comp;
        this._controller = world.createCharacterController(Math.max(0.0001, comp.skinWidth));
        this._controller.setApplyImpulsesToDynamicBodies(true);
        this._controller.setMaxSlopeClimbAngle(toRadian(comp.slopeLimit));
        this._controller.enableAutostep(comp.stepOffset, 0.01, true);

        const desc = new R.RigidBodyDesc(ERapierBodyType.KINEMATIC_POSITION_BASED as RAPIER.RigidBodyType)
            .setTranslation(this._position.x, this._position.y, this._position.z);
        this._body = world.createRigidBody(desc);
        this._collider = world.createCollider(this.buildColliderDesc(), this._body);
        this._applyFilter();
    }

    private _destroyNative (): void {
        if (this._world.destroyed) {
            // The world already freed every collider, body and controller it owned;
            // calling into it now would fault inside the bindings.
            this._collider = null;
            this._body = null;
            this._controller = null;
            return;
        }
        const world = this._world.impl;
        if (this._collider) {
            world.removeCollider(this._collider, false);
            this._collider = null;
        }
        if (this._body) {
            world.removeRigidBody(this._body);
            this._body = null;
        }
        if (this._controller) {
            world.removeCharacterController(this._controller);
            this._controller = null;
        }
    }

    private _rebuildCollider (): void {
        if (!this._collider || !this._body) return;
        this._world.impl.removeCollider(this._collider, false);
        this._collider = this._world.impl.createCollider(this.buildColliderDesc(), this._body);
        this._applyFilter();
    }

    private _pushPositionToNative (): void {
        // The body is kinematic-position-based, so `setNextKinematicTranslation` is what
        // gives it contact velocity against dynamic bodies.
        if (this._body) this._body.setNextKinematicTranslation(this._position as RAPIER.Vector);
    }

    private _applyFilter (): void {
        if (!this._collider) return;
        this._collider.setCollisionGroups(
            packInteractionGroups(this._collisionFilterGroup, this._collisionFilterMask),
        );
    }
}
