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

import type * as RAPIER from '@cocos/rapier3d-compat';
import { Mat3, Quat, Vec3, IVec3Like, geometry, warn } from '../../../core';
import { Collider, PhysicsMaterial, PhysicsSystem, RigidBody } from '../../../../exports/physics-framework';
import { IBaseShape } from '../../spec/i-physics-shape';
import { RapierWorld } from '../rapier-world';
import { RapierSharedBody } from '../rapier-shared-body';
import { RapierCache, CC_V3_0 } from '../rapier-cache';
import { ERapierActiveCollisionTypes, ERapierActiveEvents, ERapierCoefficientCombineRule, ERapierSharedBodyDirty } from '../rapier-enum';
import { packInteractionGroups } from '../rapier-utils';
import { R } from '../instantiated';

const m3_0 = new Mat3();

/**
 * Componentwise absolute value of a 3x3 matrix.
 *
 * Multiplying a half-extent vector by |R| gives the half-extents of the world-axis-aligned
 * box that encloses the rotated box, which is what an AABB needs.
 */
function absolute3x3 (m: Mat3): Mat3 {
    m.m00 = Math.abs(m.m00); m.m01 = Math.abs(m.m01); m.m02 = Math.abs(m.m02);
    m.m03 = Math.abs(m.m03); m.m04 = Math.abs(m.m04); m.m05 = Math.abs(m.m05);
    m.m06 = Math.abs(m.m06); m.m07 = Math.abs(m.m07); m.m08 = Math.abs(m.m08);
    return m;
}

let _warnedAboutRollingFriction = false;

/** @mangle */
export abstract class RapierShape implements IBaseShape {
    private static _idCounter = 0;

    /** Small dense integer, used as the key in the world's event pair dictionaries. */
    readonly id = RapierShape._idCounter++;

    protected _collider!: Collider;
    protected _sharedBody!: RapierSharedBody;
    protected _impl: RAPIER.Collider | null = null;
    protected _desc!: RAPIER.ColliderDesc;
    protected _isEnabled = false;
    protected _isTrigger = false;

    /** Shape-local orientation, driven by the `direction` axis or a plane normal. */
    protected readonly _rotation = new Quat();

    /** World scale the current shape parameters were baked against. */
    protected readonly _bakedScale = new Vec3(1, 1, 1);

    get impl (): RAPIER.Collider | null {
        return this._impl;
    }

    get collider (): Collider {
        return this._collider;
    }

    get attachedRigidBody (): RigidBody | null {
        return this._collider.attachedRigidBody;
    }

    get sharedBody (): RapierSharedBody {
        return this._sharedBody;
    }

    get isTrigger (): boolean {
        return this._isTrigger;
    }

    get isEnabled (): boolean {
        return this._isEnabled;
    }

    /**
     * Builds `this._desc`. Called once from `initialize`, before the collider exists.
     * Subclasses must not touch `this._impl` here.
     */
    protected abstract onComponentSet (): void;

    /** Re-bakes the node's world scale into the shape parameters. */
    protected abstract updateScale (): void;

    /** Local half-extents of the shape's bounding box, before world rotation. */
    protected abstract getLocalHalfExtents (out: Vec3): Vec3;

    /** Local bounding sphere radius. */
    protected abstract getLocalBoundingRadius (): number;

    initialize (com: Collider): void {
        this._collider = com;
        this._sharedBody = (PhysicsSystem.instance.physicsWorld as RapierWorld).getSharedBody(com.node);
        this._sharedBody.reference = true;
        this.onComponentSet();
    }

    // Called explicitly by `Collider.onLoad`.
    onLoad (): void {
        this.setCenter(this._collider.center);
        this.setAsTrigger(this._collider.isTrigger);
    }

    onEnable (): void {
        this._isEnabled = true;
        this._sharedBody.addShape(this);
        this._sharedBody.enabled = true;
        this.setMaterial(this._collider.sharedMaterial);
        this.updateEventListener();
    }

    onDisable (): void {
        this._isEnabled = false;
        this._sharedBody.removeShape(this);
        this._sharedBody.enabled = false;
    }

    onDestroy (): void {
        this._sharedBody.wrappedWorld.purgePairsForShape(this.id);
        // Defensive: `onDisable` normally removed the collider already, but a component
        // destroyed while disabled would otherwise leave a live collider whose recycled
        // handle could later resolve to this dead shape.
        this.destroyCollider();
        this._sharedBody.reference = false;
        (this._collider as unknown) = null;
    }

    /**
     * Creates the underlying Rapier collider on the shared body and registers its handle.
     * Called by `RapierSharedBody.addShape`.
     */
    createCollider (body: RAPIER.RigidBody): void {
        if (this._impl) return;
        const world = this._sharedBody.wrappedWorld.impl;
        this._impl = world.createCollider(this._desc, body);
        RapierCache.setShape(this._impl.handle, this);
        this.applyStateToCollider();
    }

    /** Removes the collider from the world, deregistering its handle first. */
    destroyCollider (): void {
        if (!this._impl) return;
        RapierCache.delShape(this._impl.handle);
        this._sharedBody.wrappedWorld.impl.removeCollider(this._impl, true);
        this._impl = null;
    }

    /**
     * Rebuilds the collider from `this._desc`, preserving everything that lives on the
     * collider rather than the desc. Needed by shapes whose geometry cannot be resized in
     * place (trimesh, convex hull, plane).
     *
     * The rebuild mints a fresh handle, so the old one must be deregistered and any
     * pending event pairs purged, otherwise a recycled handle would resolve to this shape
     * through a stale entry.
     */
    protected rebuildCollider (): void {
        if (!this._impl) return;
        const body = this._sharedBody.impl;
        this._sharedBody.wrappedWorld.purgePairsForShape(this.id);
        this.destroyCollider();
        this.createCollider(body);
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    /** Re-applies wrapper state that lives on the collider, after create or rebuild. */
    protected applyStateToCollider (): void {
        if (!this._impl) return;
        this._impl.setSensor(this._isTrigger);
        this._impl.setCollisionGroups(packInteractionGroups(
            this._sharedBody.collisionFilterGroup,
            this._sharedBody.collisionFilterMask,
        ));
        this.setCenter(this._collider.center);
        this.setMaterial(this._collider.sharedMaterial);
        this.updateEventListener();
    }

    setCenter (v: IVec3Like): void {
        Vec3.multiply(CC_V3_0, v, this._collider.node.worldScale);
        this._desc.setTranslation(CC_V3_0.x, CC_V3_0.y, CC_V3_0.z);
        this._desc.setRotation(this._rotation);
        if (this._impl) {
            this._impl.setTranslationWrtParent(CC_V3_0);
            this._impl.setRotationWrtParent(this._rotation);
        }
    }

    setAsTrigger (v: boolean): void {
        if (this._isTrigger === v) return;
        this._isTrigger = v;
        this._desc.setSensor(v);
        if (this._impl) this._impl.setSensor(v);
        // A sensor must not contribute mass: Cocos treats triggers as massless, whereas
        // Rapier would otherwise derive mass from the sensor's shape.
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
        this.updateEventListener();
    }

    setMaterial (v: PhysicsMaterial | null): void {
        const mat = v ?? PhysicsSystem.instance.defaultMaterial;
        if (!mat) return;
        this._desc.setFriction(mat.friction);
        this._desc.setRestitution(mat.restitution);
        if (this._impl) {
            this._impl.setFriction(mat.friction);
            this._impl.setRestitution(mat.restitution);
            // Bullet multiplies the two coefficients; matching that keeps this backend
            // numerically closest to the behaviour 3.8 projects are tuned against.
            // Rapier's own default would be Average.
            this._impl.setFrictionCombineRule(ERapierCoefficientCombineRule.MULTIPLY);
            this._impl.setRestitutionCombineRule(ERapierCoefficientCombineRule.MULTIPLY);
        }
        if (!_warnedAboutRollingFriction && (mat.rollingFriction !== 0 || mat.spinningFriction !== 0)) {
            _warnedAboutRollingFriction = true;
            warn('[PHYSICS][rapier]: rollingFriction and spinningFriction have no equivalent in '
                + 'Rapier and are ignored. Rolling bodies will not come to rest from friction alone.');
        }
    }

    /**
     * Rapier only reports collisions for a pair when at least one collider has
     * COLLISION_EVENTS active, so the framework's event-subscription state has to be
     * pushed down here.
     *
     * Event-subscribed colliders also need ActiveCollisionTypes.ALL: the default excludes
     * fixed/fixed and kinematic/fixed pairs, which would silently drop the very common
     * case of a kinematic character entering a static trigger volume. Restricting ALL to
     * subscribed colliders keeps that extra broad-phase cost off everything else.
     */
    updateEventListener (): void {
        const world = this._sharedBody?.wrappedWorld;
        if (!world) return;
        const needs = this._collider.needCollisionEvent || this._collider.needTriggerEvent;
        if (this._impl) {
            this._impl.setActiveEvents(needs ? ERapierActiveEvents.COLLISION_EVENTS : ERapierActiveEvents.NONE);
            this._impl.setActiveCollisionTypes(needs
                ? ERapierActiveCollisionTypes.ALL
                : ERapierActiveCollisionTypes.DEFAULT);
        }
        world.updateNeedEmitEvents(needs);
    }

    /*
     * Group and mask live on the shared body, not on the individual shape, because that is
     * how the Cocos framework models them: both `IRigidBody` and `IBaseShape` expose
     * `IGroupMask` over the same underlying pair. The shared body fans the packed value out
     * to each of its Rapier colliders. This mirrors the PhysX backend's `updateFilterData`.
     */
    setGroup (v: number): void {
        this._sharedBody.collisionFilterGroup = v;
    }

    getGroup (): number {
        return this._sharedBody.collisionFilterGroup;
    }

    addGroup (v: number): void {
        this._sharedBody.collisionFilterGroup |= v;
    }

    removeGroup (v: number): void {
        this._sharedBody.collisionFilterGroup &= ~v;
    }

    setMask (v: number): void {
        this._sharedBody.collisionFilterMask = v;
    }

    getMask (): number {
        return this._sharedBody.collisionFilterMask;
    }

    addMask (v: number): void {
        this._sharedBody.collisionFilterMask |= v;
    }

    removeMask (v: number): void {
        this._sharedBody.collisionFilterMask &= ~v;
    }

    getAABB (v: geometry.AABB): void {
        this.getLocalHalfExtents(CC_V3_0);
        Mat3.fromQuat(m3_0, this._collider.node.worldRotation);
        absolute3x3(m3_0);
        Vec3.transformMat3(v.halfExtents, CC_V3_0, m3_0);
        Vec3.add(v.center, this._collider.node.worldPosition, this._collider.center);
    }

    getBoundingSphere (v: geometry.Sphere): void {
        v.radius = this.getLocalBoundingRadius();
        Vec3.add(v.center, this._collider.node.worldPosition, this._collider.center);
    }

    /** Called by the shared body when the node's scale changed. */
    syncScale (): void {
        const ws = this._collider.node.worldScale;
        if (Vec3.equals(this._bakedScale, ws)) return;
        Vec3.copy(this._bakedScale, ws);
        this.updateScale();
        this.setCenter(this._collider.center);
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    /** Clamped world scale, guarding against a zero-volume shape. */
    protected get worldScale (): Readonly<Vec3> {
        return this._collider.node.worldScale;
    }

    protected clampSize (v: number): number {
        return Math.max(Math.abs(v), PhysicsSystem.instance.minVolumeSize);
    }

    /** Guard so subclasses can assert the module is live before touching `R`. */
    protected get rapier (): typeof R {
        return R;
    }
}
