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
import { IQuatLike, IVec3Like, RecyclePool, Vec3, geometry, warn } from '../../core';
import { Node } from '../../scene-graph';
import { PhysicsMaterial, PhysicsRayResult } from '../../../exports/physics-framework';
import { EPhysicsDrawFlags } from '../framework/physics-enum';
import { CollisionEventType, TriggerEventType } from '../framework/physics-interface';
import { IPhysicsWorld, IRaycastOptions } from '../spec/i-physics-world';
import { TupleDictionary } from '../utils/tuple-dictionary';
import { CollisionEventObject, TriggerEventObject } from '../utils/util';
import { RapierSharedBody } from './rapier-shared-body';
import { RapierContactEquation } from './rapier-contact-equation';
import { RapierCache, CC_V3_0, CC_V3_1 } from './rapier-cache';
import { toQueryFilterFlags, toQueryGroups } from './rapier-utils';
import { R, assertRapierReady } from './instantiated';
import type { RapierShape } from './shapes/rapier-shape';
import type { RapierRigidBody } from './rapier-rigid-body';

/** A pair of touching shapes, plus how many steps it has been touching. */
interface IRapierPairItem {
    a: RapierShape;
    b: RapierShape;
    times: number;
}

let _warnedAboutSweep = false;

/** @mangle */
export class RapierWorld implements IPhysicsWorld {
    private _world: RAPIER.World;
    private _eventQueue: RAPIER.EventQueue;

    readonly bodies: RapierSharedBody[] = [];

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
    private _needSyncAfterEvents = false;
    private _ray: RAPIER.Ray | null = null;

    private _defaultMaterial: PhysicsMaterial | null = null;
    private _debugDrawFlags: EPhysicsDrawFlags = EPhysicsDrawFlags.NONE;

    constructor () {
        assertRapierReady('new RapierWorld()');
        this._world = new R.World({ x: 0, y: -10, z: 0 });
        // autoDrain: the queue self-clears at the start of each `world.step`.
        this._eventQueue = new R.EventQueue(true);
    }

    get impl (): RAPIER.World {
        return this._world;
    }

    get debugDrawFlags (): EPhysicsDrawFlags {
        return this._debugDrawFlags;
    }

    set debugDrawFlags (v: EPhysicsDrawFlags) {
        this._debugDrawFlags = v;
    }

    get debugDrawConstraintSize (): number {
        // Constraints are not implemented yet, so there is nothing to scale.
        return 0;
    }

    set debugDrawConstraintSize (_v: number) {
        // no-op until constraints land
    }

    get defaultMaterial (): PhysicsMaterial | null {
        return this._defaultMaterial;
    }

    setGravity (v: IVec3Like): void {
        this._world.gravity = { x: v.x, y: v.y, z: v.z };
    }

    /**
     * Rapier has no world-level sleep toggle, and `setCanSleep` exists on
     * `RigidBodyDesc` only. Bodies are therefore force-woken each step instead; see
     * `RapierSharedBody.forceWakeIfNeeded`.
     */
    setAllowSleep (v: boolean): void {
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
        if (this.bodies.length === 0) return;
        this._world.timestep = deltaTime;
        this._world.step(this._eventQueue);

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
    }

    syncSceneToPhysics (): void {
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].syncSceneToPhysics();
        }
    }

    /**
     * Applies transform writes made by user code inside event callbacks. Uses the
     * checked variant so the writeback performed at the end of `step` (which itself dirties
     * `node.hasChangedFlags`) is not pushed back into the solver.
     */
    syncAfterEvents (): void {
        if (!this._needSyncAfterEvents) return;
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].syncSceneWithCheck();
        }
    }

    getSharedBody (node: Node, wrappedBody?: RapierRigidBody): RapierSharedBody {
        return RapierSharedBody.getSharedBody(node, this, wrappedBody);
    }

    addSharedBody (sharedBody: RapierSharedBody): void {
        if (this.bodies.indexOf(sharedBody) < 0) {
            this.bodies.push(sharedBody);
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
        this._needSyncAfterEvents = false;
        if (!this._needEmitEvents) {
            this._pairBeginDic.reset();
            this._pairEndDic.reset();
            return;
        }

        this._drainTransitions();
        this._emitExitPass();
        this._emitEnterStayPass();
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

    /*
     * Shape sweeps are deferred to phase 2. Rapier's `castShape` covers the *Closest
     * variants directly, but it is closest-only — there is no all-hits shape-cast
     * iterator, so the non-closest variants need a repeated-cast-with-exclusion loop.
     */
    sweepBox (): boolean {
        return this._sweepUnsupported();
    }

    sweepBoxClosest (): boolean {
        return this._sweepUnsupported();
    }

    sweepSphere (): boolean {
        return this._sweepUnsupported();
    }

    sweepSphereClosest (): boolean {
        return this._sweepUnsupported();
    }

    sweepCapsule (): boolean {
        return this._sweepUnsupported();
    }

    sweepCapsuleClosest (): boolean {
        return this._sweepUnsupported();
    }

    destroy (): void {
        // Copy first: `destroy()` mutates `this.bodies` through `removeSharedBody`.
        const bodies = this.bodies.slice();
        for (let i = 0; i < bodies.length; i++) bodies[i].destroy();
        this.bodies.length = 0;
        this._pairBeginDic.reset();
        this._pairEndDic.reset();
        this._pairPool.length = 0;
        this._contactsPool.length = 0;
        this._eventQueue.free();
        this._world.free();
    }

    /* ---------------------------------------------------------------- internals */

    private _sweepUnsupported (): boolean {
        if (!_warnedAboutSweep) {
            _warnedAboutSweep = true;
            warn('[PHYSICS][rapier]: shape sweeps are not implemented by the rapier backend yet. '
                + 'Use raycast, or switch to the bullet or physx backend.');
        }
        return false;
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
