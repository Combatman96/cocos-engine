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
import { Color, IQuatLike, IVec3Like, Quat, RecyclePool, Vec3, geometry } from '../../core';
import { director } from '../../game';
import { GeometryRenderer } from '../../rendering/geometry-renderer';
import { Node } from '../../scene-graph';
import { Collider, PhysicsMaterial, PhysicsRayResult } from '../../../exports/physics-framework';
import { EPhysicsDrawFlags } from '../framework/physics-enum';
import { CollisionEventType, TriggerEventType } from '../framework/physics-interface';
import { IPhysicsWorld, IRaycastOptions } from '../spec/i-physics-world';
import { TupleDictionary } from '../utils/tuple-dictionary';
import { CollisionEventObject, TriggerEventObject } from '../utils/util';
import { rapierContactForceEvents } from './extras/rapier-events';
import { RapierSharedBody } from './rapier-shared-body';
import { RapierContactEquation } from './rapier-contact-equation';
import { RapierCache, CC_V3_0, CC_V3_1 } from './rapier-cache';
import { toQueryFilterFlags, toQueryGroups } from './rapier-utils';
import { ERapierBodyType, RAPIER_MAX_SWEEP_HITS } from './rapier-enum';
import { R, assertRapierReady } from './instantiated';
import type { RapierShape } from './shapes/rapier-shape';
import type { RapierRigidBody } from './rapier-rigid-body';
import type { RapierConstraint } from './constraints/rapier-constraint';
import type { RapierCharacterController } from './character-controllers/rapier-character-controller';

/** A pair of touching shapes, plus how many steps it has been touching. */
interface IRapierPairItem {
    a: RapierShape;
    b: RapierShape;
    times: number;
}

/** @mangle */
export class RapierWorld implements IPhysicsWorld {
    private _world: RAPIER.World;
    private _eventQueue: RAPIER.EventQueue;

    readonly bodies: RapierSharedBody[] = [];
    readonly constraints: RapierConstraint[] = [];
    readonly ccts: RapierCharacterController[] = [];

    /**
     * Pairs currently touching. PERSISTENT across steps.
     *
     * This is the structural difference from the bullet backend, whose `contactsDic` is
     * rebuilt from the full manifold list every step and reset at the end of `emitEvents`.
     * Rapier reports only enter/exit transitions and gives no per-step list of touching
     * pairs, so the "still touching" state has to be remembered here; resetting it would
     * turn every Stay into a dropped event. This is the same approach the PhysX backend
     * takes, since PhysX is likewise enter/exit only.
     */
    private readonly _pairBeginDic = new TupleDictionary();
    /** Pairs that stopped touching this step. Drained and reset every step. */
    private readonly _pairEndDic = new TupleDictionary();

    private readonly _pairPool: IRapierPairItem[] = [];
    private readonly _contactsPool: RapierContactEquation[] = [];

    private _needEmitEvents = false;
    private _needEmitCCTEvents = false;
    private _needSyncAfterEvents = false;
    private _ray: RAPIER.Ray | null = null;
    private readonly _sweepExcluded = new Set<number>();
    private readonly _sweepPredicate = (collider: RAPIER.Collider): boolean => !this._sweepExcluded.has(collider.handle);
    private _sweepBoxShape: RAPIER.Cuboid | null = null;
    private _sweepBallShape: RAPIER.Ball | null = null;
    private _sweepCapsuleShape: RAPIER.Capsule | null = null;

    private _defaultMaterial: PhysicsMaterial | null = null;
    private _debugDrawFlags: EPhysicsDrawFlags = EPhysicsDrawFlags.NONE;
    private _debugDrawConstraintSize = 0.3;
    private _allowSleep = true;
    private _destroyed = false;
    private _anchorBody: RAPIER.RigidBody | null = null;
    private _hooks: RAPIER.PhysicsHooks | null = null;
    private readonly _MAX_DEBUG_LINE_COUNT = 16384;
    private _debugLineCount = 0;
    private readonly _aabbColor = new Color(0, 255, 255, 255);
    private readonly _wireColor = new Color(255, 255, 255, 255);
    private readonly _debugV3_0 = new Vec3();
    private readonly _debugV3_1 = new Vec3();
    private readonly _debugAABB = new geometry.AABB();

    constructor () {
        assertRapierReady('new RapierWorld()');
        this._world = new R.World({ x: 0, y: -10, z: 0 });
        // autoDrain: the queue self-clears at the start of each `world.step`.
        this._eventQueue = new R.EventQueue(true);
    }

    get impl (): RAPIER.World {
        return this._world;
    }

    /** True once `destroy()` has freed the underlying wasm world. */
    get destroyed (): boolean {
        return this._destroyed;
    }

    get debugDrawFlags (): EPhysicsDrawFlags {
        return this._debugDrawFlags;
    }

    set debugDrawFlags (v: EPhysicsDrawFlags) {
        this._debugDrawFlags = v;
    }

    get debugDrawConstraintSize (): number {
        return this._debugDrawConstraintSize;
    }

    set debugDrawConstraintSize (v: number) {
        this._debugDrawConstraintSize = v;
    }

    get defaultMaterial (): PhysicsMaterial | null {
        return this._defaultMaterial;
    }

    setGravity (v: IVec3Like): void {
        this._world.gravity = { x: v.x, y: v.y, z: v.z };
    }

    get allowSleep (): boolean {
        return this._allowSleep;
    }

    /**
     * Rapier has no world-level sleep toggle, and `setCanSleep` exists on
     * `RigidBodyDesc` only. Bodies are therefore force-woken each step instead; see
     * `RapierSharedBody.forceWakeIfNeeded`.
     *
     * The flag is stored as well as applied, because `constructDefaultWorld` calls this
     * before a single body exists; `addSharedBody` then carries it to bodies created later.
     */
    setAllowSleep (v: boolean): void {
        this._allowSleep = v;
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].setAllowSleep(v);
        }
    }

    /**
     * Rapier has no world default material, so it is remembered here and pushed onto
     * every shape that has no material of its own.
     */
    setDefaultMaterial (v: PhysicsMaterial): void {
        this._defaultMaterial = v;
        for (let i = 0; i < this.bodies.length; i++) {
            const shapes = this.bodies[i].wrappedShapes;
            for (let j = 0; j < shapes.length; j++) {
                if (!shapes[j].collider.sharedMaterial) shapes[j].setMaterial(null);
            }
        }
    }

    /**
     * `RAPIER.World.step` takes no delta; the timestep is world state.
     * Signature is `step(eventQueue?, hooks?)`.
     */
    step (deltaTime: number): void {
        // `destroy()` frees the wasm world; anything reaching it afterwards would fault
        // inside the bindings rather than raising a legible error.
        if (this._destroyed) return;
        // Set before the early return: the kinematic character controller reads
        // `integrationParameters.dt`, and a scene holding only character controllers has
        // no rigid bodies at all.
        this._world.timestep = deltaTime;
        if (this.bodies.length === 0) return;
        this._world.step(this._eventQueue, this._hooks ?? undefined);

        // Physics -> scene writeback. `IPhysicsWorld` has no `syncPhysicsToScene`, so like
        // the cannon and physx backends this happens at the tail of `step`. Iterating only
        // active bodies means sleeping and static bodies cost nothing.
        this._world.forEachActiveRigidBody((body): void => {
            if (!body.isDynamic()) return; // the scene owns kinematic and fixed transforms
            // Resolved through our own handle registry rather than `body.userData`: the
            // registry is written at creation time and is guaranteed correct, whereas
            // relying on RigidBodyDesc.userData propagating to the body would make the
            // entire writeback depend on an implementation detail of the bindings.
            const sb = RapierCache.getBody(body.handle);
            if (sb) sb.syncPhysicsToScene();
        });

        for (let i = 0; i < this.bodies.length; i++) {
            const sb = this.bodies[i];
            sb.resetForcesIfDirty();
            sb.forceWakeIfNeeded();
        }

        this._debugDraw();
    }

    syncSceneToPhysics (): void {
        if (this._destroyed) return;
        for (let i = 0; i < this.constraints.length; i++) {
            this.constraints[i].flushRebuild();
        }
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].syncSceneToPhysics();
        }
        for (let i = 0; i < this.ccts.length; i++) {
            this.ccts[i].syncSceneToPhysics();
        }
    }

    /**
     * Applies transform writes made by user code inside event callbacks. Uses the
     * checked variant so the writeback performed at the end of `step` (which itself dirties
     * `node.hasChangedFlags`) is not pushed back into the solver.
     */
    syncAfterEvents (): void {
        if (this._destroyed || !this._needSyncAfterEvents) return;
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].syncSceneWithCheck();
        }
    }

    getSharedBody (node: Node, wrappedBody?: RapierRigidBody): RapierSharedBody {
        return RapierSharedBody.getSharedBody(node, this, wrappedBody);
    }

    /**
     * A single static body every joint uses as its second parent when the Cocos
     * `connectedBody` is null. Rapier has no equivalent of Bullet's shared fixed body, and
     * `createImpulseJoint` requires two real bodies.
     */
    get anchorBody (): RAPIER.RigidBody {
        if (!this._anchorBody) {
            this._anchorBody = this._world.createRigidBody(
                new R.RigidBodyDesc(ERapierBodyType.FIXED as RAPIER.RigidBodyType),
            );
        }
        return this._anchorBody;
    }

    /**
     * Installs per-step physics hooks. Only colliders that also opt in through
     * `setActiveHooks` reach the callbacks.
     */
    setPhysicsHooks (hooks: RAPIER.PhysicsHooks | null): void {
        this._hooks = hooks;
    }

    addCCT (v: RapierCharacterController): void {
        if (this.ccts.indexOf(v) < 0) this.ccts.push(v);
    }

    removeCCT (v: RapierCharacterController): void {
        const i = this.ccts.indexOf(v);
        if (i >= 0) this.ccts.splice(i, 1);
    }

    /**
     * Same idempotent shape as `updateNeedEmitEvents`: turning it on is a direct set,
     * turning it off rescans, because this is called once per listener add/remove.
     */
    updateNeedEmitCCTEvents (v: boolean): void {
        if (v) {
            this._needEmitCCTEvents = true;
            return;
        }
        this._needEmitCCTEvents = false;
        for (let i = 0; i < this.ccts.length; i++) {
            const comp = this.ccts[i].characterController;
            if (comp && comp.needCollisionEvent) {
                this._needEmitCCTEvents = true;
                return;
            }
        }
    }

    addConstraint (v: RapierConstraint): void {
        if (this.constraints.indexOf(v) < 0) this.constraints.push(v);
    }

    removeConstraint (v: RapierConstraint): void {
        const i = this.constraints.indexOf(v);
        if (i >= 0) this.constraints.splice(i, 1);
    }

    addSharedBody (sharedBody: RapierSharedBody): void {
        if (this.bodies.indexOf(sharedBody) < 0) {
            this.bodies.push(sharedBody);
            sharedBody.setAllowSleep(this._allowSleep);
        }
    }

    removeSharedBody (sharedBody: RapierSharedBody): void {
        const i = this.bodies.indexOf(sharedBody);
        if (i >= 0) this.bodies.splice(i, 1);
    }

    /**
     * Tracks whether any shape still wants events, so `emitEvents` can bail cheaply.
     *
     * Turning it on is a direct set; turning it off rescans, because this is called once
     * per listener add/remove (and again while a collider is being created), so it has to
     * be idempotent. A counter would double-count and never reach zero. Same approach as
     * `BulletWorld.updateNeedEmitEvents`.
     */
    updateNeedEmitEvents (v: boolean): void {
        if (v) {
            this._needEmitEvents = true;
            return;
        }
        this._needEmitEvents = false;
        for (let i = 0; i < this.bodies.length; i++) {
            const shapes = this.bodies[i].wrappedShapes;
            for (let j = 0; j < shapes.length; j++) {
                const collider = shapes[j].collider;
                if (collider && (collider.needCollisionEvent || collider.needTriggerEvent)) {
                    this._needEmitEvents = true;
                    return;
                }
            }
        }
    }

    /**
     * Drops every pending pair naming `shapeId`.
     *
     * Required whenever a collider is destroyed or rebuilt: without it the Begin entry
     * never receives its matching Stopped event, leaking a stale shape reference, and a
     * recycled handle could later resolve through it.
     */
    purgePairsForShape (shapeId: number): void {
        this._purgeFrom(this._pairBeginDic, shapeId);
        this._purgeFrom(this._pairEndDic, shapeId);
    }

    emitEvents (): void {
        if (this._destroyed) return;
        this._needSyncAfterEvents = false;
        this._emitCCTEvents();
        this._emitContactForceEvents();
        if (!this._needEmitEvents) {
            this._pairBeginDic.reset();
            this._pairEndDic.reset();
            return;
        }

        this._drainTransitions();
        this._emitExitPass();
        this._emitEnterStayPass();
    }

    private readonly _forceEvent = {
        colliderA: null as Collider | null,
        colliderB: null as Collider | null,
        totalForce: new Vec3(),
        totalForceMagnitude: 0,
        maxForceDirection: new Vec3(),
        maxForceMagnitude: 0,
    };

    /**
     * Drains Rapier's contact-force queue. Everything is copied inside the callback: the
     * event is a temporary wasm view that is freed once the callback returns.
     */
    private _emitContactForceEvents (): void {
        if (!rapierContactForceEvents._hasListeners()) return;
        this._eventQueue.drainContactForceEvents((event): void => {
            const a = RapierCache.getShape(event.collider1());
            const b = RapierCache.getShape(event.collider2());
            const e = this._forceEvent;
            e.colliderA = a ? a.collider : null;
            e.colliderB = b ? b.collider : null;
            event.totalForce(e.totalForce);
            e.totalForceMagnitude = event.totalForceMagnitude();
            event.maxForceDirection(e.maxForceDirection);
            e.maxForceMagnitude = event.maxForceMagnitude();
            rapierContactForceEvents._dispatch(e);
        });
    }

    private _emitCCTEvents (): void {
        if (!this._needEmitCCTEvents) return;
        for (let i = 0; i < this.ccts.length; i++) {
            const cct = this.ccts[i];
            const contacts = cct.pendingContacts;
            for (let j = 0; j < contacts.length; j++) {
                cct.characterController.emit('onControllerColliderHit', contacts[j]);
                this._needSyncAfterEvents = true;
            }
            cct.recycleContacts();
        }
    }

    /* ---------------------------------------------------------------- queries */

    raycastClosest (worldRay: geometry.Ray, options: IRaycastOptions, result: PhysicsRayResult): boolean {
        const ray = this._makeRay(worldRay);
        const hit = this._world.castRayAndGetNormal(
            ray,
            options.maxDistance,
            true, // solid: a ray starting inside a shape reports a hit at toi 0
            toQueryFilterFlags(options) as RAPIER.QueryFilterFlags,
            toQueryGroups(options),
        );
        if (!hit) return false;
        const shape = RapierCache.getShape(hit.collider.handle);
        if (!shape) return false;
        this._assignHit(result, ray, hit.timeOfImpact, hit.normal, shape, options.maxDistance);
        return true;
    }

    raycast (
        worldRay: geometry.Ray,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        const ray = this._makeRay(worldRay);
        let any = false;
        this._world.intersectionsWithRay(
            ray,
            options.maxDistance,
            true,
            (hit): boolean => {
                const shape = RapierCache.getShape(hit.collider.handle);
                if (shape) {
                    any = true;
                    const r = pool.add();
                    this._assignHit(r, ray, hit.timeOfImpact, hit.normal, shape, options.maxDistance);
                    results.push(r);
                }
                return true; // keep going: collect every hit
            },
            toQueryFilterFlags(options) as RAPIER.QueryFilterFlags,
            toQueryGroups(options),
        );
        return any;
    }

    sweepBox (
        worldRay: geometry.Ray,
        halfExtent: IVec3Like,
        orientation: IQuatLike,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        return this._sweepAll(this._boxShape(halfExtent), orientation, worldRay, options, pool, results);
    }

    sweepBoxClosest (
        worldRay: geometry.Ray,
        halfExtent: IVec3Like,
        orientation: IQuatLike,
        options: IRaycastOptions,
        result: PhysicsRayResult,
    ): boolean {
        return this._sweepClosest(this._boxShape(halfExtent), orientation, worldRay, options, result);
    }

    sweepSphere (
        worldRay: geometry.Ray,
        radius: number,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        return this._sweepAll(this._ballShape(radius), Quat.IDENTITY, worldRay, options, pool, results);
    }

    sweepSphereClosest (
        worldRay: geometry.Ray,
        radius: number,
        options: IRaycastOptions,
        result: PhysicsRayResult,
    ): boolean {
        return this._sweepClosest(this._ballShape(radius), Quat.IDENTITY, worldRay, options, result);
    }

    sweepCapsule (
        worldRay: geometry.Ray,
        radius: number,
        height: number,
        orientation: IQuatLike,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        return this._sweepAll(this._capsuleShape(radius, height), orientation, worldRay, options, pool, results);
    }

    sweepCapsuleClosest (
        worldRay: geometry.Ray,
        radius: number,
        height: number,
        orientation: IQuatLike,
        options: IRaycastOptions,
        result: PhysicsRayResult,
    ): boolean {
        return this._sweepClosest(this._capsuleShape(radius, height), orientation, worldRay, options, result);
    }

    destroy (): void {
        if (this._destroyed) return;
        this._destroyed = true;

        const ccts = this.ccts.slice();
        for (let i = 0; i < ccts.length; i++) ccts[i].onDestroy();
        this.ccts.length = 0;

        // Joints must go before bodies: removing a rigid body first would leave the joint
        // holding a freed parent handle.
        const constraints = this.constraints.slice();
        for (let i = 0; i < constraints.length; i++) constraints[i].destroyJoint();
        this.constraints.length = 0;

        // Copy first: `destroy()` mutates `this.bodies` through `removeSharedBody`.
        const bodies = this.bodies.slice();
        for (let i = 0; i < bodies.length; i++) bodies[i].destroy();
        this.bodies.length = 0;
        this._pairBeginDic.reset();
        this._pairEndDic.reset();
        this._pairPool.length = 0;
        this._contactsPool.length = 0;
        this._anchorBody = null;
        this._eventQueue.free();
        this._world.free();
    }

    /* ---------------------------------------------------------------- debug draw */

    /** Same accessor every other backend uses; null when there is no camera yet. */
    private _getDebugRenderer (): GeometryRenderer | null {
        const cameras = director.root!.mainWindow?.cameras;
        if (!cameras) return null;
        if (cameras.length === 0) return null;
        if (!cameras[0]) return null;
        cameras[0].initGeometryRenderer();

        return cameras[0].geometryRenderer;
    }

    private _debugDraw (): void {
        if (this._debugDrawFlags === EPhysicsDrawFlags.NONE) return;
        const renderer = this._getDebugRenderer();
        if (!renderer) return;
        this._debugLineCount = 0;

        if (this._debugDrawFlags & EPhysicsDrawFlags.WIRE_FRAME) {
            // Rapier renders the entire world in a single call, which makes this the
            // cheapest debug draw of any backend: a flat line list of 3 floats per vertex,
            // 2 vertices per line, plus one RGBA colour per vertex.
            const buffers = this._world.debugRender();
            const verts = buffers.vertices;
            const colors = buffers.colors;
            for (let i = 0; i + 5 < verts.length; i += 6) {
                if (this._debugLineCount >= this._MAX_DEBUG_LINE_COUNT) break;
                this._debugLineCount++;
                this._debugV3_0.set(verts[i], verts[i + 1], verts[i + 2]);
                this._debugV3_1.set(verts[i + 3], verts[i + 4], verts[i + 5]);
                const c = (i / 6) * 8;
                if (c + 3 < colors.length) {
                    this._wireColor.set(
                        colors[c] * 255,
                        colors[c + 1] * 255,
                        colors[c + 2] * 255,
                        colors[c + 3] * 255,
                    );
                }
                renderer.addLine(this._debugV3_0, this._debugV3_1, this._wireColor);
            }
        }

        if (this._debugDrawFlags & EPhysicsDrawFlags.AABB) {
            const AABB_LINE_COUNT = 12;
            for (let i = 0; i < this.bodies.length; i++) {
                const shapes = this.bodies[i].wrappedShapes;
                for (let j = 0; j < shapes.length; j++) {
                    if (this._debugLineCount + AABB_LINE_COUNT >= this._MAX_DEBUG_LINE_COUNT) break;
                    this._debugLineCount += AABB_LINE_COUNT;
                    shapes[j].getAABB(this._debugAABB);
                    renderer.addBoundingBox(this._debugAABB, this._aabbColor);
                }
            }
        }
    }

    /* ---------------------------------------------------------------- internals */

    /* Cached sweep shapes. These are plain JS objects, so mutating them costs nothing. */
    private _boxShape (halfExtent: IVec3Like): RAPIER.Shape {
        if (!this._sweepBoxShape) this._sweepBoxShape = new R.Cuboid(halfExtent.x, halfExtent.y, halfExtent.z);
        this._sweepBoxShape.halfExtents = halfExtent as RAPIER.Vector;
        return this._sweepBoxShape;
    }

    private _ballShape (radius: number): RAPIER.Shape {
        if (!this._sweepBallShape) this._sweepBallShape = new R.Ball(radius);
        this._sweepBallShape.radius = radius;
        return this._sweepBallShape;
    }

    private _capsuleShape (radius: number, height: number): RAPIER.Shape {
        // Rapier's capsule half-height excludes the hemispherical caps, whereas the Cocos
        // sweep API passes the total height.
        const halfHeight = Math.max(0, height * 0.5 - radius);
        if (!this._sweepCapsuleShape) this._sweepCapsuleShape = new R.Capsule(halfHeight, radius);
        this._sweepCapsuleShape.halfHeight = halfHeight;
        this._sweepCapsuleShape.radius = radius;
        return this._sweepCapsuleShape;
    }

    /**
     * Normalizes the sweep direction into CC_V3_1 and reports whether it is usable.
     * `geometry.Ray.d` is not guaranteed to be unit length, and a zero vector would make
     * `Vec3.normalize` yield NaN that Rapier propagates into `time_of_impact`.
     */
    private _prepareSweepDir (worldRay: geometry.Ray): boolean {
        if (Vec3.lengthSqr(worldRay.d) < 1e-12) return false;
        Vec3.normalize(CC_V3_1, worldRay.d);
        return true;
    }

    private _castShapeOnce (
        shape: RAPIER.Shape,
        orientation: IQuatLike,
        worldRay: geometry.Ray,
        options: IRaycastOptions,
        usePredicate: boolean,
    ): RAPIER.ColliderShapeCastHit | null {
        return this._world.castShape(
            worldRay.o as RAPIER.Vector,
            orientation as RAPIER.Rotation,
            CC_V3_1 as RAPIER.Vector,
            shape,
            0,
            options.maxDistance,
            true,
            toQueryFilterFlags(options) as RAPIER.QueryFilterFlags,
            toQueryGroups(options),
            undefined,
            undefined,
            usePredicate ? this._sweepPredicate : undefined,
        );
    }

    private _sweepClosest (
        shape: RAPIER.Shape,
        orientation: IQuatLike,
        worldRay: geometry.Ray,
        options: IRaycastOptions,
        result: PhysicsRayResult,
    ): boolean {
        if (this._destroyed || !this._prepareSweepDir(worldRay)) return false;
        const hit = this._castShapeOnce(shape, orientation, worldRay, options, false);
        if (!hit) return false;
        const wrapped = RapierCache.getShape(hit.collider.handle);
        if (!wrapped) return false;
        this._assignSweepHit(result, hit, wrapped, options.maxDistance);
        return true;
    }

    /**
     * Rapier's `castShape` is closest-only, so all-hits is a repeated cast that excludes
     * everything already found through `filterPredicate`.
     */
    private _sweepAll (
        shape: RAPIER.Shape,
        orientation: IQuatLike,
        worldRay: geometry.Ray,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        if (this._destroyed || !this._prepareSweepDir(worldRay)) return false;
        this._sweepExcluded.clear();
        let any = false;
        for (let i = 0; i < RAPIER_MAX_SWEEP_HITS; i++) {
            const hit = this._castShapeOnce(shape, orientation, worldRay, options, true);
            if (!hit) break;
            this._sweepExcluded.add(hit.collider.handle);
            const wrapped = RapierCache.getShape(hit.collider.handle);
            if (!wrapped) continue;
            any = true;
            const r = pool.add();
            this._assignSweepHit(r, hit, wrapped, options.maxDistance);
            results.push(r);
        }
        this._sweepExcluded.clear();
        return any;
    }

    private _assignSweepHit (
        out: PhysicsRayResult,
        hit: RAPIER.ColliderShapeCastHit,
        shape: RapierShape,
        maxDistance: number,
    ): void {
        const toi = hit.time_of_impact;
        // `witness1` is the contact point on the hit collider, which is what Cocos reports
        // as the hit point; the swept shape's own origin ends up at `o + dir * toi`.
        Vec3.copy(CC_V3_0, hit.witness1 as Vec3);
        out._assign(CC_V3_0, toi, shape.collider, hit.normal1 as IVec3Like, maxDistance > 0 ? toi / maxDistance : 0);
    }

    private _makeRay (worldRay: geometry.Ray): RAPIER.Ray {
        // Cached and mutated in place: raycasts run in hot loops, and `Ray` is a plain JS
        // object (no wasm allocation) whose fields we can simply repoint.
        if (!this._ray) this._ray = new R.Ray(worldRay.o, CC_V3_1);
        // Normalize so Rapier's time-of-impact comes back in world distance units, which
        // lets us use it directly as `distance`.
        Vec3.normalize(CC_V3_1, worldRay.d);
        this._ray.origin = worldRay.o;
        this._ray.dir = CC_V3_1;
        return this._ray;
    }

    private _assignHit (
        out: PhysicsRayResult,
        ray: RAPIER.Ray,
        toi: number,
        normal: RAPIER.Vector,
        shape: RapierShape,
        maxDistance: number,
    ): void {
        Vec3.scaleAndAdd(CC_V3_0, ray.origin as Vec3, ray.dir as Vec3, toi);
        // `toi` is already a distance because `dir` is normalized.
        out._assign(CC_V3_0, toi, shape.collider, normal as IVec3Like, maxDistance > 0 ? toi / maxDistance : 0);
    }

    /** Folds Rapier's enter/exit transitions into the two pair dictionaries. */
    private _drainTransitions (): void {
        this._eventQueue.drainCollisionEvents((h1, h2, started): void => {
            const a = RapierCache.getShape(h1);
            const b = RapierCache.getShape(h2);
            if (!a || !b) return; // a collider was removed during this step
            if (!this._pairWantsEvents(a, b)) return;
            if (started) {
                if (!this._pairBeginDic.get<IRapierPairItem>(a.id, b.id)) {
                    this._pairBeginDic.set(a.id, b.id, this._acquirePair(a, b));
                }
            } else {
                this._pairEndDic.set(a.id, b.id, { a, b });
            }
        });
    }

    private _emitExitPass (): void {
        const dic = this._pairEndDic;
        const keys = dic.data.keys;
        for (let i = keys.length - 1; i >= 0; i--) {
            const item = dic.getDataByKey<{ a: RapierShape; b: RapierShape }>(keys[i]);
            const { a, b } = item;
            // Drop the Begin entry so the pair does not keep emitting Stay.
            const begun = this._pairBeginDic.get<IRapierPairItem>(a.id, b.id);
            if (begun) {
                this._releasePair(begun);
                this._pairBeginDic.set<IRapierPairItem | null>(a.id, b.id, null);
            }
            const ca = a.collider;
            const cb = b.collider;
            if (!ca || !cb || !ca.isValid || !cb.isValid) continue;
            if (a.isTrigger || b.isTrigger) {
                this._emitTrigger(a, b, 'onTriggerExit');
            } else {
                this._recycleContacts();
                this._emitCollision(a, b, 'onCollisionExit', false);
            }
            this._needSyncAfterEvents = true;
        }
        dic.reset();
    }

    private _emitEnterStayPass (): void {
        const dic = this._pairBeginDic;
        const keys = dic.data.keys;
        // Reverse walk: an entry may be removed mid-iteration when a collider went away.
        for (let i = keys.length - 1; i >= 0; i--) {
            const item = dic.getDataByKey<IRapierPairItem>(keys[i]);
            if (!item) continue;
            const { a, b } = item;
            const ca = a.collider;
            const cb = b.collider;
            if (!ca || !cb || !ca.isValid || !cb.isValid) {
                this._releasePair(item);
                dic.set<IRapierPairItem | null>(a.id, b.id, null);
                continue;
            }
            // Nothing is moving, so there is no new information to report.
            if (this._pairIsAsleep(a, b)) continue;

            if (a.isTrigger || b.isTrigger) {
                const type: TriggerEventType = item.times++ ? 'onTriggerStay' : 'onTriggerEnter';
                this._emitTrigger(a, b, type);
            } else {
                const type: CollisionEventType = item.times++ ? 'onCollisionStay' : 'onCollisionEnter';
                this._recycleContacts();
                this._gatherContacts(a, b);
                this._emitCollision(a, b, type, true);
            }
            this._needSyncAfterEvents = true;
        }
    }

    /**
     * Reads the pair's contact manifolds into pooled `RapierContactEquation`s.
     *
     * Everything must be copied inside the callback: the manifold is a reused view whose
     * wasm memory is invalidated once the callback returns.
     */
    private _gatherContacts (a: RapierShape, b: RapierShape): void {
        const ia = a.impl;
        const ib = b.impl;
        if (!ia || !ib) return;
        this._world.contactPair(ia, ib, (manifold, flipped): void => {
            const n = manifold.numContacts();
            for (let i = 0; i < n; i++) {
                const c = this._contactsPool.pop() ?? new RapierContactEquation(CollisionEventObject);
                c.flipped = flipped;
                c.isBodyA = !flipped;
                manifold.localContactPoint1(i, c.localPointA);
                manifold.localContactPoint2(i, c.localPointB);
                manifold.localNormal1(c.localNormalA);
                manifold.localNormal2(c.localNormalB);
                manifold.normal(c.worldNormal);
                c.distance = manifold.contactDist(i);
                c.impulse = manifold.contactImpulse(i);
                ia.translation(c.worldPosA);
                ia.rotation(c.worldRotA);
                ib.translation(c.worldPosB);
                ib.rotation(c.worldRotB);
                CollisionEventObject.contacts.push(c);
            }
        });
    }

    private _emitTrigger (a: RapierShape, b: RapierShape, type: TriggerEventType): void {
        TriggerEventObject.type = type;
        TriggerEventObject.impl = null;
        // Emit to each side independently; a side only hears about it if it subscribed.
        if (a.collider.needTriggerEvent) {
            TriggerEventObject.selfCollider = a.collider;
            TriggerEventObject.otherCollider = b.collider;
            a.collider.emit(type, TriggerEventObject);
        }
        if (b.collider.needTriggerEvent) {
            TriggerEventObject.selfCollider = b.collider;
            TriggerEventObject.otherCollider = a.collider;
            b.collider.emit(type, TriggerEventObject);
        }
    }

    private _emitCollision (
        a: RapierShape,
        b: RapierShape,
        type: CollisionEventType,
        hasContacts: boolean,
    ): void {
        CollisionEventObject.type = type;
        CollisionEventObject.impl = null;
        if (a.collider.needCollisionEvent) {
            CollisionEventObject.selfCollider = a.collider;
            CollisionEventObject.otherCollider = b.collider;
            if (hasContacts) this._orientContacts(true);
            a.collider.emit(type, CollisionEventObject);
        }
        if (b.collider.needCollisionEvent) {
            CollisionEventObject.selfCollider = b.collider;
            CollisionEventObject.otherCollider = a.collider;
            if (hasContacts) this._orientContacts(false);
            b.collider.emit(type, CollisionEventObject);
        }
    }

    /**
     * Re-points each contact's `isBodyA` at whichever collider is about to receive the
     * event, so that "A" always means "me" from the listener's perspective — the same
     * convention the cannon backend uses.
     *
     * `flipped` must be folded in rather than ignored: it records whether the manifold's
     * collider1/collider2 are swapped relative to the (a, b) pair, so for recipient `a`
     * the answer is `!flipped`, and for recipient `b` it is `flipped`. Overwriting
     * `isBodyA` with a bare true/false would silently hand back the other collider's
     * contact points and an inverted normal for every flipped manifold.
     */
    private _orientContacts (selfIsPairFirst: boolean): void {
        const contacts = CollisionEventObject.contacts as RapierContactEquation[];
        for (let i = 0; i < contacts.length; i++) {
            const c = contacts[i];
            c.isBodyA = selfIsPairFirst ? !c.flipped : c.flipped;
        }
    }

    private _recycleContacts (): void {
        const contacts = CollisionEventObject.contacts as RapierContactEquation[];
        for (let i = 0; i < contacts.length; i++) this._contactsPool.push(contacts[i]);
        contacts.length = 0;
    }

    private _pairWantsEvents (a: RapierShape, b: RapierShape): boolean {
        const ca = a.collider;
        const cb = b.collider;
        if (!ca || !cb) return false;
        return ca.needTriggerEvent || cb.needTriggerEvent || ca.needCollisionEvent || cb.needCollisionEvent;
    }

    private _pairIsAsleep (a: RapierShape, b: RapierShape): boolean {
        const ba = a.sharedBody;
        const bb = b.sharedBody;
        if (!ba.hasBody || !bb.hasBody) return false;
        return ba.impl.isSleeping() && bb.impl.isSleeping();
    }

    private _acquirePair (a: RapierShape, b: RapierShape): IRapierPairItem {
        const item = this._pairPool.pop();
        if (item) {
            item.a = a;
            item.b = b;
            item.times = 0;
            return item;
        }
        return { a, b, times: 0 };
    }

    private _releasePair (item: IRapierPairItem): void {
        this._pairPool.push(item);
    }

    private _purgeFrom (dic: TupleDictionary, shapeId: number): void {
        const keys = dic.data.keys;
        for (let i = keys.length - 1; i >= 0; i--) {
            const key = keys[i];
            const dash = key.indexOf('-');
            const i0 = parseInt(key.substring(0, dash), 10);
            const j0 = parseInt(key.substring(dash + 1), 10);
            if (i0 === shapeId || j0 === shapeId) {
                const item = dic.getDataByKey<IRapierPairItem>(key);
                if (item && typeof item.times === 'number') this._releasePair(item);
                dic.set<IRapierPairItem | null>(i0, j0, null);
            }
        }
    }
}
