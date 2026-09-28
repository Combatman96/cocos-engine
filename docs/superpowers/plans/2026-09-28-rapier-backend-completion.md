# Rapier Physics Backend Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the Rapier 3D physics backend so it fills every `IPhysicsWrapperObject` slot, passes all ten `tests/physics` suites, and exposes Rapier-only capabilities the Cocos spec cannot express.

**Architecture:** The backend lives in `cocos/physics/rapier/` and plugs into the engine through one `selector.register('rapier', {...})` call. Phase 1 (world, rigid body, 7 shapes, raycast, events) already ships and passes 11 tests. This plan hardens that base, then adds shape sweeps, the four constraints, both character controllers, terrain/simplex shapes, debug draw, the editor feature entry, and a tree-shakeable `extras/` API for Rapier-only features.

**Tech Stack:** TypeScript 4.9, `@dimforge/rapier3d-compat` 0.20.0 (WASM, wasm-bindgen), jest + jsdom, `@cocos/ccbuild`.

**Spec:** `cocos/physics/rapier/README.md`

## Global Constraints

Copied from the spec; every task's requirements implicitly include this section.

- Dependency pinned exactly: `"@dimforge/rapier3d-compat": "0.20.0"`. Never widen to a caret range — Rapier breaks API every minor.
- Target is **web / H5 only**. No JSB, no CMake, no mini-game subpackaging, no `isNativeModule` / `cmakeConfig` in editor config.
- **Never dereference `R` at module-evaluation time.** Rapier classes and enums exist only after `init()` resolves. Touch `R` from constructors and methods only. Numeric enums must be mirrored in `cocos/physics/rapier/rapier-enum.ts`.
- **Rapier handles are recycled arena indices.** Every collider/body removal and every rebuild must call `RapierCache.delShape` / `delBody` synchronously before anything new is created.
- Every new `.ts` file starts with the 23-line Cocos MIT licence header copied verbatim from `cocos/physics/rapier/rapier-world.ts`.
- Every exported backend class carries a `/** @mangle */` doc comment, matching the bullet backend.
- `npx tsc --noEmit` must report **0 errors repo-wide** after every task.
- `npx eslint "cocos/physics/rapier/**/*.ts"` must be clean after every task. The repo uses single quotes and 4-space indent.
- `physics-cannon` must remain the **last** physics import in `tests/physics/physics.test.ts`.
- Angles crossing the boundary: Cocos components use **degrees**, Rapier uses **radians**. Convert with `toRadian` / `toDegree` from `cocos/core`.

## Review Focus

Input classes the spec implies but no task's own happy-path test exercises. Each line's test is added to the owning task.

1. **Unnormalized or zero-length sweep/ray direction** (Task 1) — `geometry.Ray.d` is not guaranteed unit length, and a zero vector makes `Vec3.normalize` produce `NaN`, which Rapier propagates into `time_of_impact`. Expected: a zero direction returns no hit rather than `NaN` results.
2. **Constraint whose `connectedBody` is null, then set, then nulled again** (Task 2) — Rapier needs two real bodies, so null maps to the world anchor body. Expected: the joint rebinds cleanly each time without leaking the previous `ImpulseJoint`.
3. **`CharacterController.move` below `minMoveDistance`, and with a zero `fixedTimeStep`** (Task 6) — the framework passes `minDist` and `elapsedTime` straight through. Expected: sub-threshold moves are a no-op and a zero timestep does not divide by zero.
4. **Non-square terrain (`sizeI !== sizeJ`)** (Task 8) — Rapier's heightfield buffer is **column-major** while Cocos reads `getHeight(i, j)` row-major, so a transposition bug is invisible on square terrain. Expected: heights land at the correct world XZ.
5. **Constraint or CCT attached to a node whose `RigidBody` is disabled or destroyed mid-life** (Task 2, Task 6) — the joint holds a `RAPIER.RigidBody` reference that `removeRigidBody` invalidates. Expected: the joint is removed before the body, and no call lands on a freed handle.

---

## Task 0: Harden the existing backend

Six defects found auditing the shipped phase-1 code. Two are load-bearing for later tasks: the double-destroy crash gets harder to isolate once constraints also hold body references, and the timestep bug silently breaks character controllers in a scene with no rigid bodies.

**Files:**
- Modify: `cocos/physics/rapier/rapier-shared-body.ts`
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Modify: `cocos/physics/rapier/shapes/rapier-shape.ts`
- Test: `tests/physics/rapier-internals.test.ts` (create)

**Interfaces:**
- Consumes: nothing.
- Produces: `RapierWorld.debugDrawConstraintSize` becomes a real read/write field (Task 9 sets it). `RapierSharedBody.inWorld: boolean` replaces the `index` number.

- [ ] **Step 1: Write the failing test**

Create `tests/physics/rapier-internals.test.ts`:

```ts
import { director, game, Game } from "../../cocos/game";
import { physics, PhysicsSystem } from "../../exports/physics-framework";
import "../../exports/physics-rapier";
import "../../exports/physics-cannon";
import { waitForRapierInstantiation } from "../../cocos/physics/rapier/instantiated";
import { Node, Scene } from "../../cocos/scene-graph";
import { Vec3 } from "../../cocos/core";

beforeAll(async () => {
    await waitForRapierInstantiation();
    game.emit(Game.EVENT_PRE_SUBSYSTEM_INIT);
    PhysicsSystem.constructAndRegister();
});

describe('rapier internals', () => {
    let scene: Scene;

    beforeEach(() => {
        physics.selector.switchTo('rapier');
        scene = new Scene('rapier-internals');
        director.runSceneImmediate(scene);
    });

    afterEach(() => {
        scene.destroy();
    });

    test('world.destroy() then component teardown does not double-destroy', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        node.addComponent(physics.RigidBody);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // Destroys every shared body directly, bypassing the ref count.
        PhysicsSystem.instance.physicsWorld.destroy();
        // Component teardown then drives the ref count to zero a second time.
        expect(() => { node.destroy(); }).not.toThrow();
    });

    test('step sets the timestep even with no rigid bodies', () => {
        const world = PhysicsSystem.instance.physicsWorld as any;
        world.impl.timestep = 0.5;
        world.step(1 / 90);
        expect(world.impl.timestep).toBeCloseTo(1 / 90, 6);
    });

    test('destroying a collider deregisters its handle', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const shape = (collider as any)._shape;
        const handle = shape.impl.handle as number;
        const { RapierCache } = require('../../cocos/physics/rapier/rapier-cache');

        expect(RapierCache.getShape(handle)).toBe(shape);
        node.destroy();
        expect(RapierCache.getShape(handle)).toBeUndefined();
    });

    test('allowSleep set before any body exists still reaches later bodies', () => {
        const world = PhysicsSystem.instance.physicsWorld;
        world.setAllowSleep(false);

        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.ERigidBodyType.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const sharedBody = (rb as any)._body.sharedBody;
        expect(sharedBody.allowSleep).toBe(false);
    });

    test('debugDrawConstraintSize round-trips', () => {
        const world = PhysicsSystem.instance.physicsWorld;
        world.debugDrawConstraintSize = 0.75;
        expect(world.debugDrawConstraintSize).toBe(0.75);
    });
});
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts`
Expected: FAIL. `does not double-destroy` throws `TypeError: Cannot read properties of null (reading 'uuid')`; `timestep` is still `0.5`; `debugDrawConstraintSize` is `0` not `0.75`.

- [ ] **Step 3: Add the re-entry guard and drop the stale index**

In `cocos/physics/rapier/rapier-shared-body.ts`, replace the `_index` field and its accessors:

```ts
    private _inWorld = false;
    private _destroyed = false;
```

Delete the `get index` / `set index` accessors and add:

```ts
    get inWorld (): boolean {
        return this._inWorld;
    }
```

Rewrite the `enabled` setter to use the boolean:

```ts
    set enabled (v: boolean) {
        if (this._destroyed) return;
        if (v) {
            if (!this._inWorld) {
                this._inWorld = true;
                this.wrappedWorld.addSharedBody(this);
                this.syncInitial();
                this.impl.setEnabled(true);
            }
        } else if (this._inWorld) {
            const isRemove = (this.wrappedShapes.length === 0 && this._wrappedBody === null)
                || (this.wrappedShapes.length === 0 && this._wrappedBody !== null && !this._wrappedBody.isEnabled);
            if (isRemove) {
                this.clearVelocity();
                if (this._body) this._body.setEnabled(false);
                this._inWorld = false;
                this.wrappedWorld.removeSharedBody(this);
            }
        }
    }
```

Guard `reference` and `destroy`:

```ts
    set reference (v: boolean) {
        if (this._destroyed) return;
        // eslint-disable-next-line @typescript-eslint/no-unused-expressions
        v ? this._ref++ : this._ref--;
        if (this._ref <= 0) this.destroy();
    }

    destroy (): void {
        // `RapierWorld.destroy()` destroys bodies directly, bypassing the ref count, so a
        // later component teardown can reach here a second time. Without this guard the
        // second pass dereferences the already-nulled node.
        if (this._destroyed) return;
        this._destroyed = true;

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
        if (this._inWorld) {
            this._inWorld = false;
            this.wrappedWorld.removeSharedBody(this);
        }
        this._wrappedBody = null;
        (this.node as unknown) = null;
    }
```

- [ ] **Step 4: Fix the timestep, allowSleep and constraint size on the world**

In `cocos/physics/rapier/rapier-world.ts`, add backing fields next to `_debugDrawFlags`:

```ts
    private _debugDrawConstraintSize = 0.3;
    private _allowSleep = true;
```

Replace the `debugDrawConstraintSize` accessors:

```ts
    get debugDrawConstraintSize (): number {
        return this._debugDrawConstraintSize;
    }

    set debugDrawConstraintSize (v: number) {
        this._debugDrawConstraintSize = v;
    }
```

Replace `setAllowSleep` so the value survives to bodies created later:

```ts
    get allowSleep (): boolean {
        return this._allowSleep;
    }

    /**
     * Rapier has no world-level sleep toggle, and `setCanSleep` exists on `RigidBodyDesc`
     * only. The flag is stored and force-waking is done per step; `addSharedBody` applies
     * it to bodies created after this call, which is the normal order because
     * `constructDefaultWorld` runs before any body exists.
     */
    setAllowSleep (v: boolean): void {
        this._allowSleep = v;
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].setAllowSleep(v);
        }
    }
```

Apply it on insertion:

```ts
    addSharedBody (sharedBody: RapierSharedBody): void {
        if (this.bodies.indexOf(sharedBody) < 0) {
            this.bodies.push(sharedBody);
            sharedBody.setAllowSleep(this._allowSleep);
        }
    }
```

Hoist the timestep above the early return in `step`:

```ts
    step (deltaTime: number): void {
        // Set before the early return: `KinematicCharacterController` reads
        // `integrationParameters.dt`, and a scene with only character controllers has no
        // rigid bodies at all.
        this._world.timestep = deltaTime;
        if (this.bodies.length === 0) return;
        this._world.step(this._eventQueue);
        // ... unchanged remainder
```

- [ ] **Step 5: Close the collider handle leak**

In `cocos/physics/rapier/shapes/rapier-shape.ts`, replace `onDestroy`:

```ts
    onDestroy (): void {
        this._sharedBody.wrappedWorld.purgePairsForShape(this.id);
        // Defensive: normally `onDisable` already removed the collider, but a component
        // destroyed while disabled would otherwise leave a live collider whose recycled
        // handle could resolve to this dead shape.
        this.destroyCollider();
        this._sharedBody.reference = false;
        (this._collider as unknown) = null;
    }
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `npx jest tests/physics/rapier-internals.test.ts`
Expected: PASS, 5 tests.

- [ ] **Step 7: Verify no regression and commit**

Run: `npx jest tests/physics && npx tsc --noEmit && npx eslint "cocos/physics/rapier/**/*.ts"`
Expected: 61+ tests pass, 0 type errors, clean lint.

```bash
git add cocos/physics/rapier tests/physics/rapier-internals.test.ts
git commit -m "fix(physics): harden rapier backend against double-destroy and handle leaks"
```

---

## Task 1: Shape sweeps

Rapier's `castShape` is closest-only. The all-hits variants need a repeated cast that excludes already-hit colliders through `filterPredicate`.

**Files:**
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Modify: `cocos/physics/rapier/rapier-enum.ts`
- Modify: `tests/physics/physics.test.ts:89-92`
- Test: `tests/physics/sweep.ts` (existing, unmodified)

**Interfaces:**
- Consumes: `RapierCache.getShape`, `toQueryFilterFlags`, `toQueryGroups` from Task 0's untouched modules.
- Produces: `RapierWorld.sweepBox/sweepBoxClosest/sweepSphere/sweepSphereClosest/sweepCapsule/sweepCapsuleClosest` matching `IPhysicsWorld`.

- [ ] **Step 1: Write the failing test**

Delete the rapier sweep guard in `tests/physics/physics.test.ts`, restoring the original call:

```ts
    SweepTest(env);
```

Add a zero-direction regression to `tests/physics/rapier-internals.test.ts`, inside `describe('rapier internals', ...)`:

```ts
    test('sweep with a zero-length direction returns no hit and no NaN', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const { geometry, Quat } = require('../../cocos/core');
        const ray = new geometry.Ray(0, 0, 0, 0, 0, 0);
        const hit = PhysicsSystem.instance.sweepBoxClosest(ray, new Vec3(0.5, 0.5, 0.5), new Quat());
        expect(hit).toBe(false);
        expect(Number.isNaN(PhysicsSystem.instance.sweepCastClosestResult.distance)).toBe(false);
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics -t "Sweep" && npx jest tests/physics/rapier-internals.test.ts -t "zero-length"`
Expected: FAIL. `Sweep` fails `expect(isHit).toBe(true)` (currently returns false and warns); the zero-length test fails because the warning path also returns false but for the wrong reason — it will pass only once real sweeps exist.

- [ ] **Step 3: Add the sweep implementation**

In `cocos/physics/rapier/rapier-enum.ts`, append the cap used by the all-hits loop:

```ts
/**
 * Upper bound on the repeated-cast loop used for all-hits shape sweeps. Rapier's
 * `castShape` is closest-only, so each iteration excludes the colliders already found.
 */
export const RAPIER_MAX_SWEEP_HITS = 64;
```

In `cocos/physics/rapier/rapier-world.ts`, add imports and scratch state:

```ts
import { IQuatLike, Quat } from '../../core';
import { RAPIER_MAX_SWEEP_HITS } from './rapier-enum';
```

```ts
    private _sweepExcluded = new Set<number>();
    private _sweepPredicate = (collider: RAPIER.Collider): boolean => !this._sweepExcluded.has(collider.handle);
    private _sweepBox: RAPIER.Cuboid | null = null;
    private _sweepBall: RAPIER.Ball | null = null;
    private _sweepCapsule: RAPIER.Capsule | null = null;
```

Replace the six stub methods and `_sweepUnsupported` with:

```ts
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

    /** Cached shape instances; these are plain JS objects, so mutating them is free. */
    private _boxShape (halfExtent: IVec3Like): RAPIER.Shape {
        if (!this._sweepBox) this._sweepBox = new R.Cuboid(halfExtent.x, halfExtent.y, halfExtent.z);
        this._sweepBox.halfExtents = halfExtent;
        return this._sweepBox;
    }

    private _ballShape (radius: number): RAPIER.Shape {
        if (!this._sweepBall) this._sweepBall = new R.Ball(radius);
        this._sweepBall.radius = radius;
        return this._sweepBall;
    }

    private _capsuleShape (radius: number, height: number): RAPIER.Shape {
        // Rapier's capsule half-height excludes the hemispherical caps, whereas the Cocos
        // sweep API passes the total height.
        const halfHeight = Math.max(0, height * 0.5 - radius);
        if (!this._sweepCapsule) this._sweepCapsule = new R.Capsule(halfHeight, radius);
        this._sweepCapsule.halfHeight = halfHeight;
        this._sweepCapsule.radius = radius;
        return this._sweepCapsule;
    }

    /**
     * Normalizes the sweep direction into CC_V3_1 and reports whether it is usable.
     * A zero-length `geometry.Ray.d` would otherwise produce NaN components that Rapier
     * propagates straight into `time_of_impact`.
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
            worldRay.o,
            orientation as RAPIER.Rotation,
            CC_V3_1,
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
        if (!this._prepareSweepDir(worldRay)) return false;
        const hit = this._castShapeOnce(shape, orientation, worldRay, options, false);
        if (!hit) return false;
        const wrapped = RapierCache.getShape(hit.collider.handle);
        if (!wrapped) return false;
        this._assignSweepHit(result, worldRay, hit, wrapped, options.maxDistance);
        return true;
    }

    private _sweepAll (
        shape: RAPIER.Shape,
        orientation: IQuatLike,
        worldRay: geometry.Ray,
        options: IRaycastOptions,
        pool: RecyclePool<PhysicsRayResult>,
        results: PhysicsRayResult[],
    ): boolean {
        if (!this._prepareSweepDir(worldRay)) return false;
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
            this._assignSweepHit(r, worldRay, hit, wrapped, options.maxDistance);
            results.push(r);
        }
        this._sweepExcluded.clear();
        return any;
    }

    private _assignSweepHit (
        out: PhysicsRayResult,
        worldRay: geometry.Ray,
        hit: RAPIER.ColliderShapeCastHit,
        shape: RapierShape,
        maxDistance: number,
    ): void {
        const toi = hit.time_of_impact;
        // `witness1` is the contact point on the hit collider, which is what Cocos reports
        // as the hit point; the swept shape's own origin is at `o + dir * toi`.
        Vec3.copy(CC_V3_0, hit.witness1 as Vec3);
        out._assign(CC_V3_0, toi, shape.collider, hit.normal1 as IVec3Like, maxDistance > 0 ? toi / maxDistance : 0);
    }
```

Delete the now-unused `_warnedAboutSweep` module-level flag.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `npx jest tests/physics -t "Sweep" && npx jest tests/physics/rapier-internals.test.ts`
Expected: PASS. `Sweep` reports 2 hits for `sweepBox` and the box for `sweepBoxClosest`.

- [ ] **Step 5: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): implement shape sweeps for the rapier backend"
```

---

## Task 2: Constraint base, world anchor body, point-to-point

Rapier's `createImpulseJoint` needs two real bodies and offers no `getFixedBody()`, so a null `connectedBody` binds to a lazily created static anchor body owned by the world.

**Files:**
- Create: `cocos/physics/rapier/constraints/rapier-constraint.ts`
- Create: `cocos/physics/rapier/constraints/rapier-p2p-constraint.ts`
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierSharedBody.impl`, `RapierWorld.impl`.
- Produces: `abstract RapierConstraint implements IBaseConstraint` with `protected abstract buildJointData(): RAPIER.JointData`, `protected scheduleRebuild(): void`, `get impl(): RAPIER.ImpulseJoint | null`. `RapierWorld.anchorBody: RAPIER.RigidBody`, `RapierWorld.addConstraint/removeConstraint`, `RapierWorld.constraints: RapierConstraint[]`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)` in `tests/physics/rapier-internals.test.ts`:

```ts
    test('point-to-point constraint binds to the anchor body when connectedBody is null', () => {
        const node = new Node('anchored');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.ERigidBodyType.DYNAMIC;
        const c = node.addComponent(physics.PointToPointConstraint) as physics.PointToPointConstraint;
        c.pivotA = new Vec3(0, 1, 0);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const impl = (c as any)._constraint.impl;
        expect(impl).not.toBeNull();
        expect(impl.isValid()).toBe(true);
    });

    test('rebinding connectedBody does not leak the previous joint', () => {
        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.ERigidBodyType.DYNAMIC;
        rbB.type = physics.ERigidBodyType.DYNAMIC;
        const c = a.addComponent(physics.PointToPointConstraint) as physics.PointToPointConstraint;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const world = (PhysicsSystem.instance.physicsWorld as any).impl;
        const before = world.impulseJoints.len();

        c.connectedBody = rbB;
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        c.connectedBody = null;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        expect(world.impulseJoints.len()).toBe(before);
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "constraint"`
Expected: FAIL with `TypeError: Cannot read properties of undefined (reading 'impl')` — `PointToPointConstraint` resolves to the selector's no-op stub because the slot is unregistered.

- [ ] **Step 3: Add the anchor body and constraint registry to the world**

In `cocos/physics/rapier/rapier-world.ts`:

```ts
import type { RapierConstraint } from './constraints/rapier-constraint';
```

```ts
    readonly constraints: RapierConstraint[] = [];
    private _anchorBody: RAPIER.RigidBody | null = null;

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

    addConstraint (v: RapierConstraint): void {
        if (this.constraints.indexOf(v) < 0) this.constraints.push(v);
    }

    removeConstraint (v: RapierConstraint): void {
        const i = this.constraints.indexOf(v);
        if (i >= 0) this.constraints.splice(i, 1);
    }
```

Import `ERapierBodyType` from `./rapier-enum`. Flush pending joint rebuilds at the top of `syncSceneToPhysics`, before body sync:

```ts
    syncSceneToPhysics (): void {
        for (let i = 0; i < this.constraints.length; i++) {
            this.constraints[i].flushRebuild();
        }
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].syncSceneToPhysics();
        }
    }
```

Extend `destroy()` so joints go before bodies — removing a body first would invalidate the joint's parents:

```ts
    destroy (): void {
        const constraints = this.constraints.slice();
        for (let i = 0; i < constraints.length; i++) constraints[i].destroyJoint();
        this.constraints.length = 0;

        const bodies = this.bodies.slice();
        // ... unchanged remainder
        this._anchorBody = null;
    }
```

- [ ] **Step 4: Write the constraint base class**

Create `cocos/physics/rapier/constraints/rapier-constraint.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Vec3 } from '../../../core';
import { Constraint, PhysicsSystem, RigidBody } from '../../../../exports/physics-framework';
import { IBaseConstraint } from '../../spec/i-physics-constraint';
import { RapierWorld } from '../rapier-world';
import { RapierRigidBody } from '../rapier-rigid-body';
import { R } from '../instantiated';

const v3_0 = new Vec3();

/** @mangle */
export abstract class RapierConstraint implements IBaseConstraint {
    protected _com!: Constraint;
    protected _rigidBody!: RigidBody;
    protected _connectedBody: RigidBody | null = null;
    protected _collided = false;
    protected _impl: RAPIER.ImpulseJoint | null = null;
    protected _world!: RapierWorld;
    private _needRebuild = false;
    private _enabled = false;

    get impl (): RAPIER.ImpulseJoint | null {
        return this._impl;
    }

    get constraint (): Constraint {
        return this._com;
    }

    /** Builds the descriptor for the current component state. */
    protected abstract buildJointData (): RAPIER.JointData;

    initialize (v: Constraint): void {
        this._com = v;
        this._rigidBody = v.attachedBody!;
        this._connectedBody = v.connectedBody;
        this._collided = v.enableCollision;
        this._world = PhysicsSystem.instance.physicsWorld as RapierWorld;
    }

    onEnable (): void {
        this._enabled = true;
        this._world.addConstraint(this);
        this.createJoint();
    }

    onDisable (): void {
        this._enabled = false;
        this.destroyJoint();
        this._world.removeConstraint(this);
    }

    onDestroy (): void {
        this.destroyJoint();
        this._world.removeConstraint(this);
        (this._com as unknown) = null;
        (this._rigidBody as unknown) = null;
        this._connectedBody = null;
    }

    setConnectedBody (v: RigidBody | null): void {
        if (this._connectedBody === v) return;
        this._connectedBody = v;
        this.scheduleRebuild();
    }

    setEnableCollision (v: boolean): void {
        if (this._collided === v) return;
        this._collided = v;
        if (this._impl) this._impl.setContactsEnabled(v);
    }

    /**
     * Most of Rapier's joint state is descriptor-only, so a changed property means the
     * joint has to be recreated. Rebuilds are deferred to the next `syncSceneToPhysics` so
     * that setting N properties in one frame costs one rebuild, not N.
     */
    protected scheduleRebuild (): void {
        this._needRebuild = true;
    }

    flushRebuild (): void {
        if (!this._needRebuild) return;
        this._needRebuild = false;
        if (!this._enabled) return;
        this.destroyJoint();
        this.createJoint();
    }

    createJoint (): void {
        if (this._impl) return;
        const bodyA = (this._rigidBody.body as RapierRigidBody | null)?.sharedBody.impl;
        if (!bodyA) return;
        const connected = this._connectedBody;
        // A null connectedBody anchors to the world's shared static body.
        const bodyB = connected && connected.body
            ? (connected.body as RapierRigidBody).sharedBody.impl
            : this._world.anchorBody;
        this._impl = this._world.impl.createImpulseJoint(this.buildJointData(), bodyA, bodyB, true);
        this._impl.setContactsEnabled(this._collided);
    }

    destroyJoint (): void {
        if (!this._impl) return;
        this._world.impl.removeImpulseJoint(this._impl, true);
        this._impl = null;
    }

    /** Pivot scaled by the owning node's world scale, matching every other backend. */
    protected scaledPivotA (out: Vec3, pivot: Vec3): Vec3 {
        return Vec3.multiply(out, pivot, this._com.node.worldScale);
    }

    /**
     * When there is no connected body, the second anchor is body A's pivot expressed in
     * world space, because the anchor body sits at the origin with identity rotation.
     */
    protected scaledPivotB (out: Vec3, pivotA: Vec3, pivotB: Vec3): Vec3 {
        const connected = this._connectedBody;
        if (connected) {
            return Vec3.multiply(out, pivotB, connected.node.worldScale);
        }
        const node = this._com.node;
        Vec3.multiply(out, pivotA, node.worldScale);
        Vec3.transformQuat(out, out, node.worldRotation);
        return Vec3.add(out, out, node.worldPosition);
    }

    protected get rapier (): typeof R {
        return R;
    }

    protected get scratch (): Vec3 {
        return v3_0;
    }
}
```

- [ ] **Step 5: Write the point-to-point constraint**

Create `cocos/physics/rapier/constraints/rapier-p2p-constraint.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3 } from '../../../core';
import { PointToPointConstraint } from '../../../../exports/physics-framework';
import { IPointToPointConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();

/** @mangle */
export class RapierP2PConstraint extends RapierConstraint implements IPointToPointConstraint {
    get constraint (): PointToPointConstraint {
        return this._com as PointToPointConstraint;
    }

    setPivotA (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setPivotB (_v: IVec3Like): void {
        this._applyAnchors();
    }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        return this.rapier.JointData.spherical(anchorA, anchorB);
    }

    /** Anchors are the one part of a spherical joint Rapier can change in place. */
    private _applyAnchors (): void {
        if (!this._impl) return;
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        this._impl.setAnchor1(anchorA);
        this._impl.setAnchor2(anchorB);
    }
}
```

- [ ] **Step 6: Register the wrapper**

In `cocos/physics/rapier/instantiate.ts`, import and add to the `selector.register` object:

```ts
import { RapierP2PConstraint } from './constraints/rapier-p2p-constraint';
```

```ts
        PointToPointConstraint: RapierP2PConstraint,
```

Update the "deliberately left unregistered" comment to drop `PointToPointConstraint`.

- [ ] **Step 7: Run the tests to verify they pass**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "constraint" && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 8: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier constraint base and point-to-point joint"
```

---

## Task 3: Hinge constraint

`JointData.revolute` produces a `RevoluteImpulseJoint extends UnitImpulseJoint`, which is the one joint type with live `setLimits` and motor control.

**Files:**
- Create: `cocos/physics/rapier/constraints/rapier-hinge-constraint.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierConstraint` from Task 2.
- Produces: `RapierHingeConstraint implements IHingeConstraint`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('hinge constraint applies limits in radians', () => {
        const node = new Node('hinge');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.ERigidBodyType.DYNAMIC;
        const h = node.addComponent(physics.HingeConstraint) as physics.HingeConstraint;
        h.axis = new Vec3(0, 1, 0);
        h.limitEnabled = true;
        h.lowerLimit = -90;
        h.upperLimit = 90;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const impl = (h as any)._constraint.impl;
        expect(impl).not.toBeNull();
        expect(impl.limitsMin()).toBeCloseTo(-Math.PI / 2, 5);
        expect(impl.limitsMax()).toBeCloseTo(Math.PI / 2, 5);
    });
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "hinge"`
Expected: FAIL — `impl` is `undefined`, the slot is unregistered.

- [ ] **Step 3: Write the hinge constraint**

Create `cocos/physics/rapier/constraints/rapier-hinge-constraint.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3, toRadian } from '../../../core';
import { HingeConstraint, PhysicsSystem } from '../../../../exports/physics-framework';
import { IHingeConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();
const axis = new Vec3();

/** @mangle */
export class RapierHingeConstraint extends RapierConstraint implements IHingeConstraint {
    get constraint (): HingeConstraint {
        return this._com as HingeConstraint;
    }

    private get unitJoint (): RAPIER.RevoluteImpulseJoint | null {
        return this._impl as RAPIER.RevoluteImpulseJoint | null;
    }

    setPivotA (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setPivotB (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setAxis (_v: IVec3Like): void {
        // The axis is baked into the descriptor, so it cannot change in place.
        this.scheduleRebuild();
    }

    setLimitEnabled (v: boolean): void {
        // Rapier can widen or narrow limits live but cannot switch them off, so disabling
        // means rebuilding the joint from a descriptor with `limitsEnabled = false`.
        if (!v) {
            this.scheduleRebuild();
            return;
        }
        this._applyLimits();
    }

    setLowerLimit (_min: number): void {
        this._applyLimits();
    }

    setUpperLimit (_max: number): void {
        this._applyLimits();
    }

    setMotorEnabled (v: boolean): void {
        const joint = this.unitJoint;
        if (!joint) return;
        const cs = this.constraint;
        if (v) {
            joint.configureMotorVelocity(toRadian(cs.motorVelocity), 1);
            joint.setMotorMaxForce(cs.motorForceLimit);
        } else {
            joint.configureMotorVelocity(0, 0);
            joint.setMotorMaxForce(0);
        }
    }

    setMotorVelocity (v: number): void {
        const joint = this.unitJoint;
        if (!joint || !this.constraint.motorEnabled) return;
        joint.configureMotorVelocity(toRadian(v), 1);
    }

    setMotorForceLimit (v: number): void {
        const joint = this.unitJoint;
        if (!joint || !this.constraint.motorEnabled) return;
        joint.setMotorMaxForce(v);
    }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        Vec3.normalize(axis, cs.axis);
        if (Vec3.lengthSqr(axis) < 1e-12) Vec3.copy(axis, Vec3.UNIT_Y);
        const data = this.rapier.JointData.revolute(anchorA, anchorB, axis);
        if (cs.limitEnabled) {
            data.limitsEnabled = true;
            data.limits = [toRadian(cs.lowerLimit), toRadian(cs.upperLimit)];
        }
        return data;
    }

    createJoint (): void {
        super.createJoint();
        const cs = this.constraint;
        this.setMotorEnabled(cs.motorEnabled);
    }

    private _applyAnchors (): void {
        if (!this._impl) return;
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        this._impl.setAnchor1(anchorA);
        this._impl.setAnchor2(anchorB);
    }

    private _applyLimits (): void {
        const joint = this.unitJoint;
        const cs = this.constraint;
        if (!joint || !cs.limitEnabled) return;
        joint.setLimits(toRadian(cs.lowerLimit), toRadian(cs.upperLimit));
    }
}
```

- [ ] **Step 4: Register the wrapper**

In `cocos/physics/rapier/instantiate.ts`:

```ts
import { RapierHingeConstraint } from './constraints/rapier-hinge-constraint';
```

```ts
        HingeConstraint: RapierHingeConstraint,
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "hinge" && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 6: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier hinge constraint with limits and motor"
```

---

## Task 4: Fixed constraint

**Files:**
- Create: `cocos/physics/rapier/constraints/rapier-fixed-constraint.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierConstraint` from Task 2.
- Produces: `RapierFixedConstraint implements IFixedConstraint`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('fixed constraint holds two bodies rigidly', () => {
        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.worldPosition = new Vec3(0, 5, 0);
        b.worldPosition = new Vec3(1, 5, 0);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.ERigidBodyType.STATIC;
        rbB.type = physics.ERigidBodyType.DYNAMIC;
        const f = a.addComponent(physics.FixedConstraint) as physics.FixedConstraint;
        f.connectedBody = rbB;

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 60; i++) director.tick(dt);

        // Anchored to a static body, so b must not fall away under gravity.
        expect(b.worldPosition.y).toBeGreaterThan(4.5);
    });
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "fixed constraint"`
Expected: FAIL — `b.worldPosition.y` has fallen far below 4.5 because the constraint is a no-op stub.

- [ ] **Step 3: Write the fixed constraint**

Create `cocos/physics/rapier/constraints/rapier-fixed-constraint.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Quat, Vec3, warn } from '../../../core';
import { FixedConstraint } from '../../../../exports/physics-framework';
import { IFixedConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();
const frameA = new Quat();
const frameB = new Quat();

let _warnedAboutBreak = false;

/** @mangle */
export class RapierFixedConstraint extends RapierConstraint implements IFixedConstraint {
    get constraint (): FixedConstraint {
        return this._com as FixedConstraint;
    }

    /**
     * Rapier joints do not break. Bullet supports breakForce but not breakTorque, so a
     * partially unsupported constraint is established precedent; here neither applies.
     */
    setBreakForce (_v: number): void {
        this._warnOnce();
    }

    setBreakTorque (_v: number): void {
        this._warnOnce();
    }

    protected buildJointData (): RAPIER.JointData {
        const node = this._com.node;
        const connected = this._connectedBody;

        // Anchor A is the joint origin in body A's local frame; with no pivot property on
        // FixedConstraint the bodies are locked at their current relative transform.
        Vec3.set(anchorA, 0, 0, 0);
        Quat.identity(frameA);

        if (connected) {
            // Body A's world transform expressed in body B's local frame.
            Quat.invert(frameB, connected.node.worldRotation);
            Vec3.subtract(anchorB, node.worldPosition, connected.node.worldPosition);
            Vec3.transformQuat(anchorB, anchorB, frameB);
            Quat.multiply(frameB, frameB, node.worldRotation);
        } else {
            // The anchor body sits at the origin with identity rotation, so body A's world
            // transform is already expressed in its frame.
            Vec3.copy(anchorB, node.worldPosition);
            Quat.copy(frameB, node.worldRotation);
        }

        return this.rapier.JointData.fixed(anchorA, frameA, anchorB, frameB);
    }

    private _warnOnce (): void {
        if (_warnedAboutBreak) return;
        _warnedAboutBreak = true;
        warn('[PHYSICS][rapier]: breakForce and breakTorque have no equivalent in Rapier; '
            + 'fixed constraints never break.');
    }
}
```

- [ ] **Step 4: Register the wrapper**

In `cocos/physics/rapier/instantiate.ts`:

```ts
import { RapierFixedConstraint } from './constraints/rapier-fixed-constraint';
```

```ts
        FixedConstraint: RapierFixedConstraint,
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "fixed constraint" && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 6: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier fixed constraint"
```

---

## Task 5: Configurable (6-DOF) constraint

The hardest piece. `JointData.generic` takes only a locked-axis bitmask, and `GenericImpulseJoint` has **zero** members — no limits, no motors after creation. Everything the component sets is therefore stored and applied by rebuilding the descriptor.

**Files:**
- Create: `cocos/physics/rapier/constraints/rapier-configurable-constraint.ts`
- Modify: `cocos/physics/rapier/rapier-enum.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Modify: `tests/physics/physics.test.ts:108-112`
- Test: `tests/physics/constraint.ts` (existing, unmodified)

**Interfaces:**
- Consumes: `RapierConstraint` from Task 2.
- Produces: `RapierConfigurableConstraint implements IConfigurableConstraint`.

- [ ] **Step 1: Write the failing test**

In `tests/physics/physics.test.ts`, delete the rapier guard so the suite runs, leaving:

```ts
    ConstraintTest(env);

    CharacterControllerTest(env);
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics -t "Configurable constraint"`
Expected: FAIL. The selector warns `rapier physics does not support ConfigurableConstraint`, and the settings objects write into the `ENTIRE_CONSTRAINT` no-op stub.

- [ ] **Step 3: Add the joint-axis mirrors**

In `cocos/physics/rapier/rapier-enum.ts`:

```ts
/** Mirror of `RAPIER.JointAxesMask`. */
export const ERapierJointAxesMask = {
    LIN_X: 1,
    LIN_Y: 2,
    LIN_Z: 4,
    ANG_X: 8,
    ANG_Y: 16,
    ANG_Z: 32,
} as const;

/**
 * `EConfigurableAxis` index convention shared by `setConstraintMode` and `setDriverMode`:
 * 0/1/2 are linear X/Y/Z, 3 is twist, 4 is swing1, 5 is swing2.
 */
export const RAPIER_AXIS_TO_MASK = [
    ERapierJointAxesMask.LIN_X,
    ERapierJointAxesMask.LIN_Y,
    ERapierJointAxesMask.LIN_Z,
    ERapierJointAxesMask.ANG_X,
    ERapierJointAxesMask.ANG_Y,
    ERapierJointAxesMask.ANG_Z,
] as const;
```

- [ ] **Step 4: Write the configurable constraint**

Create `cocos/physics/rapier/constraints/rapier-configurable-constraint.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3, warn } from '../../../core';
import { ConfigurableConstraint, EConstraintMode, EDriverMode } from '../../../../exports/physics-framework';
import { IConfigurableConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';
import { RAPIER_AXIS_TO_MASK } from '../rapier-enum';

const anchorA = new Vec3();
const anchorB = new Vec3();
const axis = new Vec3();

let _warnedAboutLimits = false;
let _warnedAboutDrivers = false;

/**
 * 6-DOF constraint over Rapier's generic joint.
 *
 * Rapier's `JointData.generic` accepts only a bitmask of *locked* axes, and the resulting
 * `GenericImpulseJoint` exposes no members at all — no limits, no motors, no runtime
 * mutation of any kind. Consequences, all of which are documented in the backend README:
 *
 * - `EConstraintMode.LOCKED` is honoured exactly.
 * - `EConstraintMode.FREE` is honoured exactly (the axis is simply not in the mask).
 * - `EConstraintMode.LIMITED` degrades to FREE, with a one-time warning.
 * - Every driver mode degrades to DISABLED, with a one-time warning.
 * - Soft-constraint stiffness, damping and restitution have no effect.
 *
 * All setters store to the component and schedule a rebuild, because the descriptor is the
 * only place this state can live.
 */
/** @mangle */
export class RapierConfigurableConstraint extends RapierConstraint implements IConfigurableConstraint {
    private _lockedMask = 0;

    get constraint (): ConfigurableConstraint {
        return this._com as ConfigurableConstraint;
    }

    setConstraintMode (idx: number, v: EConstraintMode): void {
        const bit = RAPIER_AXIS_TO_MASK[idx];
        if (bit === undefined) return;
        if (v === EConstraintMode.LOCKED) {
            this._lockedMask |= bit;
        } else {
            this._lockedMask &= ~bit;
            if (v === EConstraintMode.LIMITED) this._warnLimits();
        }
        this.scheduleRebuild();
    }

    setLinearLimit (_idx: number, _lower: number, _upper: number): void {
        this._warnLimits();
    }

    setAngularExtent (_twist: number, _swing1: number, _swing2: number): void {
        this._warnLimits();
    }

    setLinearRestitution (_v: number): void { this._warnLimits(); }
    setSwingRestitution (_v: number): void { this._warnLimits(); }
    setTwistRestitution (_v: number): void { this._warnLimits(); }
    setLinearSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setLinearStiffness (_v: number): void { this._warnLimits(); }
    setLinearDamping (_v: number): void { this._warnLimits(); }
    setSwingSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setTwistSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setSwingStiffness (_v: number): void { this._warnLimits(); }
    setSwingDamping (_v: number): void { this._warnLimits(); }
    setTwistStiffness (_v: number): void { this._warnLimits(); }
    setTwistDamping (_v: number): void { this._warnLimits(); }

    setDriverMode (_idx: number, v: EDriverMode): void {
        if (v !== EDriverMode.DISABLED) this._warnDrivers();
    }

    setLinearMotorTarget (_v: IVec3Like): void { this._warnDrivers(); }
    setLinearMotorVelocity (_v: IVec3Like): void { this._warnDrivers(); }
    setLinearMotorForceLimit (_v: number): void { this._warnDrivers(); }
    setAngularMotorTarget (_v: IVec3Like): void { this._warnDrivers(); }
    setAngularMotorVelocity (_v: IVec3Like): void { this._warnDrivers(); }
    setAngularMotorForceLimit (_v: number): void { this._warnDrivers(); }

    setPivotA (_v: IVec3Like): void { this.scheduleRebuild(); }
    setPivotB (_v: IVec3Like): void { this.scheduleRebuild(); }
    setAutoPivotB (_v: boolean): void { this.scheduleRebuild(); }
    setAxis (_v: IVec3Like): void { this.scheduleRebuild(); }
    setSecondaryAxis (_v: IVec3Like): void { this.scheduleRebuild(); }

    setBreakForce (_v: number): void { /* Rapier joints do not break. */ }
    setBreakTorque (_v: number): void { /* Rapier joints do not break. */ }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        if (cs.autoPivotB) {
            // Auto pivot B means "wherever body A's pivot currently is in world space".
            this.scaledPivotB(anchorB, cs.pivotA, cs.pivotA);
        } else {
            this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        }
        Vec3.normalize(axis, cs.axis);
        if (Vec3.lengthSqr(axis) < 1e-12) Vec3.copy(axis, Vec3.UNIT_Y);
        return this.rapier.JointData.generic(anchorA, anchorB, axis, this._lockedMask as RAPIER.JointAxesMask);
    }

    private _warnLimits (): void {
        if (_warnedAboutLimits) return;
        _warnedAboutLimits = true;
        warn('[PHYSICS][rapier]: Rapier\'s generic joint supports only free and locked axes. '
            + 'LIMITED axes behave as FREE, and limit softness, stiffness, damping and '
            + 'restitution have no effect.');
    }

    private _warnDrivers (): void {
        if (_warnedAboutDrivers) return;
        _warnedAboutDrivers = true;
        warn('[PHYSICS][rapier]: Rapier\'s generic joint exposes no motors, so '
            + 'ConfigurableConstraint drivers have no effect.');
    }
}
```

- [ ] **Step 5: Register the wrapper**

In `cocos/physics/rapier/instantiate.ts`:

```ts
import { RapierConfigurableConstraint } from './constraints/rapier-configurable-constraint';
```

```ts
        ConfigurableConstraint: RapierConfigurableConstraint,
```

Update the unregistered-slots comment to list only `TerrainShape`, `SimplexShape` and the two character controllers.

- [ ] **Step 6: Run the test to verify it passes**

Run: `npx jest tests/physics -t "Configurable constraint"`
Expected: PASS. The suite is a pure property round-trip on the component, so the wrapper only has to exist and accept every setter without throwing.

- [ ] **Step 7: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier configurable constraint over the generic joint"
```

---

## Task 6: Character controllers

Rapier's controller drives a **Collider**, not a body, and applies no gravity — which matches what the Cocos suite asserts. The wrapper owns its own position and writes it back to the node.

**Files:**
- Create: `cocos/physics/rapier/character-controllers/rapier-character-controller.ts`
- Create: `cocos/physics/rapier/character-controllers/rapier-box-character-controller.ts`
- Create: `cocos/physics/rapier/character-controllers/rapier-capsule-character-controller.ts`
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Test: `tests/physics/character-controller.ts` (existing, unmodified), `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierWorld.impl`, `RapierCache`, `packInteractionGroups`.
- Produces: `abstract RapierCharacterController implements IBaseCharacterController` with `protected abstract buildColliderDesc(): RAPIER.ColliderDesc`. `RapierWorld.ccts: RapierCharacterController[]`, `addCCT`, `removeCCT`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('character controller ignores sub-threshold moves and zero timesteps', () => {
        const node = new Node('cct');
        scene.addChild(node);
        const cct = node.addComponent(physics.CapsuleCharacterController) as physics.CapsuleCharacterController;
        cct.minMoveDistance = 0.001;
        cct.centerWorldPosition = new Vec3(0, 10, 0);

        cct.move(new Vec3(0, 0.0001, 0));
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        expect(Vec3.equals(cct.centerWorldPosition as Vec3, new Vec3(0, 10, 0))).toBe(true);

        const impl = (cct as any)._cct;
        expect(() => { impl.move(new Vec3(0, 1, 0), 0.001, 0); }).not.toThrow();
        expect(Number.isNaN(cct.centerWorldPosition.y)).toBe(false);
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics -t "Character controller" && npx jest tests/physics/rapier-internals.test.ts -t "character controller"`
Expected: FAIL — the selector warns `rapier physics does not support CapsuleCharacterController`, `_cct.initialize` returns `0` from the stub so `_isInitialized` is false, and `centerWorldPosition` never moves.

- [ ] **Step 3: Add CCT bookkeeping to the world**

In `cocos/physics/rapier/rapier-world.ts`:

```ts
import type { RapierCharacterController } from './character-controllers/rapier-character-controller';
```

```ts
    readonly ccts: RapierCharacterController[] = [];

    addCCT (v: RapierCharacterController): void {
        if (this.ccts.indexOf(v) < 0) this.ccts.push(v);
    }

    removeCCT (v: RapierCharacterController): void {
        const i = this.ccts.indexOf(v);
        if (i >= 0) this.ccts.splice(i, 1);
    }
```

Sync character controllers alongside bodies in `syncSceneToPhysics`, after the constraint flush:

```ts
        for (let i = 0; i < this.ccts.length; i++) {
            this.ccts[i].syncSceneToPhysics();
        }
```

Destroy them in `destroy()`, before the bodies loop:

```ts
        const ccts = this.ccts.slice();
        for (let i = 0; i < ccts.length; i++) ccts[i].onDestroy();
        this.ccts.length = 0;
```

- [ ] **Step 4: Write the character controller base**

Create `cocos/physics/rapier/character-controllers/rapier-character-controller.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3, toRadian } from '../../../core';
import { TransformBit } from '../../../scene-graph/node-enum';
import { CharacterController, PhysicsSystem } from '../../../../exports/physics-framework';
import { IBaseCharacterController } from '../../spec/i-character-controller';
import { PhysicsGroup } from '../../framework/physics-enum';
import { RapierWorld } from '../rapier-world';
import { ERapierBodyType } from '../rapier-enum';
import { packInteractionGroups, toQueryGroups } from '../rapier-utils';
import { R } from '../instantiated';

const v3_0 = new Vec3();
const v3_1 = new Vec3();

/**
 * Rapier's `KinematicCharacterController` sweeps a **collider**, not a rigid body, and it
 * never applies gravity. The wrapper therefore owns the character's world position: it
 * keeps a kinematic body carrying the shape so other bodies can collide with it, asks the
 * controller how far a requested move may go, and writes the result back to the node.
 */
/** @mangle */
export abstract class RapierCharacterController implements IBaseCharacterController {
    protected _comp!: CharacterController;
    protected _world!: RapierWorld;
    protected _controller: RAPIER.KinematicCharacterController | null = null;
    protected _body: RAPIER.RigidBody | null = null;
    protected _collider: RAPIER.Collider | null = null;

    private _position = new Vec3();
    private _grounded = false;
    private _isEnabled = false;
    private _collisionFilterGroup: number = PhysicsGroup.DEFAULT;
    private _collisionFilterMask = -1;

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
        // Autostep needs a minimum width; half the character's smallest extent is a safe
        // default and matches what the other backends do implicitly.
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
     * `minDist` and `elapsedTime` come straight from the framework. A move shorter than
     * `minDist` is a no-op, matching every other backend, and a zero `elapsedTime` must not
     * produce a division — the controller does not use dt, so it is simply ignored here.
     */
    move (movement: IVec3Like, minDist: number, _elapsedTime: number): void {
        if (!this._isEnabled || !this._controller || !this._collider) return;
        Vec3.copy(v3_0, movement);
        if (Vec3.lengthSqr(v3_0) < minDist * minDist) return;

        this._controller.computeColliderMovement(
            this._collider,
            v3_0,
            undefined,
            toQueryGroups({
                mask: this._collisionFilterMask,
                group: this._collisionFilterGroup,
                queryTrigger: false,
                maxDistance: 0,
            }),
        );
        this._controller.computedMovement(v3_1);
        this._grounded = this._controller.computedGrounded();
        Vec3.add(this._position, this._position, v3_1);
        this._pushPositionToNative();
    }

    syncPhysicsToScene (): void {
        Vec3.subtract(v3_0, this._position, this.scaledCenter);
        this._comp.node.setWorldPosition(v3_0);
    }

    /** Teleports the controller when the node was moved by user code. */
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
        this._world.updateNeedEmitCCTEvents(this._comp.needCollisionEvent);
    }

    /** Rebuilds the shape after a scale change; the concrete class supplies the desc. */
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
        if (this._body) this._body.setNextKinematicTranslation(this._position);
    }

    private _applyFilter (): void {
        if (!this._collider) return;
        this._collider.setCollisionGroups(
            packInteractionGroups(this._collisionFilterGroup, this._collisionFilterMask),
        );
    }
}
```

Add the matching gate to `cocos/physics/rapier/rapier-world.ts`:

```ts
    private _needEmitCCTEvents = false;

    updateNeedEmitCCTEvents (v: boolean): void {
        if (v) {
            this._needEmitCCTEvents = true;
            return;
        }
        this._needEmitCCTEvents = false;
        for (let i = 0; i < this.ccts.length; i++) {
            if (this.ccts[i].characterController.needCollisionEvent) {
                this._needEmitCCTEvents = true;
                return;
            }
        }
    }
```

- [ ] **Step 5: Write the box and capsule controllers**

Create `cocos/physics/rapier/character-controllers/rapier-box-character-controller.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { BoxCharacterController } from '../../../../exports/physics-framework';
import { IBoxCharacterController } from '../../spec/i-character-controller';
import { RapierCharacterController } from './rapier-character-controller';

/** @mangle */
export class RapierBoxCharacterController extends RapierCharacterController implements IBoxCharacterController {
    get component (): BoxCharacterController {
        return this._comp as BoxCharacterController;
    }

    setHalfHeight (_value: number): void {
        this.updateScale();
    }

    setHalfSideExtent (_value: number): void {
        this.updateScale();
    }

    setHalfForwardExtent (_value: number): void {
        this.updateScale();
    }

    protected buildColliderDesc (): RAPIER.ColliderDesc {
        const comp = this.component;
        const ws = comp.node.worldScale;
        // Side extent is X, height is Y, forward extent is Z.
        return this.rapier.ColliderDesc.cuboid(
            Math.max(1e-5, comp.halfSideExtent * Math.abs(ws.x)),
            Math.max(1e-5, comp.halfHeight * Math.abs(ws.y)),
            Math.max(1e-5, comp.halfForwardExtent * Math.abs(ws.z)),
        );
    }
}
```

Create `cocos/physics/rapier/character-controllers/rapier-capsule-character-controller.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { CapsuleCharacterController } from '../../../../exports/physics-framework';
import { ICapsuleCharacterController } from '../../spec/i-character-controller';
import { RapierCharacterController } from './rapier-character-controller';

/** @mangle */
export class RapierCapsuleCharacterController extends RapierCharacterController implements ICapsuleCharacterController {
    get component (): CapsuleCharacterController {
        return this._comp as CapsuleCharacterController;
    }

    setRadius (_value: number): void {
        this.updateScale();
    }

    setHeight (_value: number): void {
        this.updateScale();
    }

    protected buildColliderDesc (): RAPIER.ColliderDesc {
        const comp = this.component;
        const ws = comp.node.worldScale;
        const radius = Math.max(1e-5, comp.radius * Math.max(Math.abs(ws.x), Math.abs(ws.z)));
        // The Cocos component's `height` is the distance between the two sphere centres,
        // which is exactly Rapier's capsule half-height doubled.
        const halfHeight = Math.max(1e-5, comp.height * 0.5 * Math.abs(ws.y));
        return this.rapier.ColliderDesc.capsule(halfHeight, radius);
    }
}
```

- [ ] **Step 6: Register the wrappers**

In `cocos/physics/rapier/instantiate.ts`:

```ts
import { RapierBoxCharacterController } from './character-controllers/rapier-box-character-controller';
import { RapierCapsuleCharacterController } from './character-controllers/rapier-capsule-character-controller';
```

```ts
        BoxCharacterController: RapierBoxCharacterController,
        CapsuleCharacterController: RapierCapsuleCharacterController,
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `npx jest tests/physics -t "Character controller" && npx jest tests/physics/rapier-internals.test.ts && npx tsc --noEmit`
Expected: PASS for both `Box character controller` and `Capsule character, controller`, plus the sub-threshold regression. 0 type errors.

- [ ] **Step 8: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier box and capsule character controllers"
```

---

## Task 7: Character-controller collision events

`onControllerColliderHit` is fed from `numComputedCollisions()` / `computedCollision(i)` right after each sweep. Trigger overlap events are not supported by Rapier's controller and stay unimplemented.

**Files:**
- Modify: `cocos/physics/rapier/character-controllers/rapier-character-controller.ts`
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierWorld.ccts`, `RapierCache.getShape`.
- Produces: `RapierCharacterController.drainCollisions()`, `RapierWorld.emitCCTEvents()`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('character controller emits onControllerColliderHit', () => {
        const floor = new Node('floor');
        scene.addChild(floor);
        floor.worldPosition = new Vec3(0, 0, 0);
        const box = floor.addComponent(physics.BoxCollider) as physics.BoxCollider;
        box.size = new Vec3(10, 1, 10);

        const node = new Node('cct');
        scene.addChild(node);
        const cct = node.addComponent(physics.CapsuleCharacterController) as physics.CapsuleCharacterController;
        cct.centerWorldPosition = new Vec3(0, 3, 0);

        let hits = 0;
        cct.on('onControllerColliderHit', () => { hits++; });

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 30; i++) {
            cct.move(new Vec3(0, -0.5, 0));
            director.tick(dt);
        }
        expect(hits).toBeGreaterThan(0);
    });
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "onControllerColliderHit"`
Expected: FAIL, `hits` is `0` — nothing drains the controller's collision list.

- [ ] **Step 3: Record collisions in the controller**

In `cocos/physics/rapier/character-controllers/rapier-character-controller.ts`, add imports and state:

```ts
import { CharacterControllerContact } from '../../framework/physics-interface';
import { RapierCache } from '../rapier-cache';
```

```ts
    /** Contacts produced by the most recent sweep, drained by the world after the step. */
    readonly pendingContacts: CharacterControllerContact[] = [];
    private _contactPool: CharacterControllerContact[] = [];
    private _collisionScratch: RAPIER.CharacterCollision | null = null;
```

At the end of `move`, after `_pushPositionToNative()`:

```ts
        if (this._comp.needCollisionEvent) this._collectCollisions(v3_0);
```

Add the collector:

```ts
    /**
     * Copies this sweep's collisions out of Rapier. `computedCollision` hands back a
     * reused view whose backing memory the next call overwrites, so every field must be
     * copied before moving on.
     */
    private _collectCollisions (motion: Vec3): void {
        if (!this._controller) return;
        const n = this._controller.numComputedCollisions();
        const motionLength = Vec3.len(motion);
        Vec3.normalize(v3_0, motion);
        for (let i = 0; i < n; i++) {
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
            contact.motionDirection.set(v3_0.x, v3_0.y, v3_0.z);
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
```

- [ ] **Step 4: Emit from the world**

In `cocos/physics/rapier/rapier-world.ts`, call the emitter at the top of `emitEvents`, before the collider passes:

```ts
    emitEvents (): void {
        this._needSyncAfterEvents = false;
        this._emitCCTEvents();
        if (!this._needEmitEvents) {
            this._pairBeginDic.reset();
            this._pairEndDic.reset();
            return;
        }
        // ... unchanged remainder
```

```ts
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
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "onControllerColliderHit" && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 6: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): emit onControllerColliderHit from rapier character controllers"
```

---

## Task 8: Terrain and simplex shapes

Rapier's heightfield buffer is **column-major** while Cocos reads `getHeight(i, j)` row-major, and the field is centred on its local origin where Cocos anchors at a corner.

**Files:**
- Create: `cocos/physics/rapier/shapes/rapier-terrain-shape.ts`
- Create: `cocos/physics/rapier/shapes/rapier-simplex-shape.ts`
- Modify: `cocos/physics/rapier/rapier-enum.ts`
- Modify: `cocos/physics/rapier/instantiate.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierShape` base from Task 0.
- Produces: `RapierTerrainShape implements ITerrainShape`, `RapierSimplexShape implements ISimplexShape`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('non-square terrain places heights at the correct XZ', () => {
        const sizeI = 3;
        const sizeJ = 5;
        // Height depends only on j, so a transposition shows up as a wrong hit height.
        const asset = {
            _uuid: 'test-terrain',
            tileSize: 1,
            getVertexCountI: () => sizeI,
            getVertexCountJ: () => sizeJ,
            getHeight: (_i: number, j: number) => j,
        };

        const node = new Node('terrain');
        scene.addChild(node);
        const tc = node.addComponent(physics.TerrainCollider) as physics.TerrainCollider;
        tc.terrain = asset as any;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const shape = (tc as any)._shape;
        expect(shape.impl).not.toBeNull();
        // Column-major: index = j * sizeI + i.
        expect(shape.heights[2 * sizeI + 0]).toBeCloseTo(2, 5);
        expect(shape.heights[4 * sizeI + 0]).toBeCloseTo(4, 5);
    });

    test('simplex collider builds a convex hull from its vertex count', () => {
        const node = new Node('simplex');
        scene.addChild(node);
        const sc = node.addComponent(physics.SimplexCollider) as physics.SimplexCollider;
        sc.shapeType = 4; // TETRAHEDRON
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        expect((sc as any)._shape.impl).not.toBeNull();
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "terrain" && npx jest tests/physics/rapier-internals.test.ts -t "simplex"`
Expected: FAIL — the selector warns `rapier physics does not support TerrainCollider` / `SimplexCollider` and `_shape.impl` is undefined on the stub.

- [ ] **Step 3: Add the heightfield flag mirror**

In `cocos/physics/rapier/rapier-enum.ts`:

```ts
/** Mirror of `RAPIER.HeightFieldFlags`. */
export const ERapierHeightFieldFlags = {
    FIX_INTERNAL_EDGES: 1,
} as const;
```

- [ ] **Step 4: Write the terrain shape**

Create `cocos/physics/rapier/shapes/rapier-terrain-shape.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3 } from '../../../core';
import { TerrainCollider } from '../../../../exports/physics-framework';
import { ITerrainShape } from '../../spec/i-physics-shape';
import { ITerrainAsset } from '../../spec/i-external';
import { RapierShape } from './rapier-shape';
import { ERapierHeightFieldFlags } from '../rapier-enum';

const offset = new Vec3();

/** @mangle */
export class RapierTerrainShape extends RapierShape implements ITerrainShape {
    /** Column-major height buffer, exposed for tests. */
    heights: Float32Array = new Float32Array(0);

    private _sizeI = 0;
    private _sizeJ = 0;
    private _tileSize = 1;
    private readonly _localOffset = new Vec3();

    get collider (): TerrainCollider {
        return this._collider as TerrainCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._readTerrain(this.collider.terrain);
        this._desc = this._buildDesc();
    }

    setTerrain (v: ITerrainAsset | null): void {
        this._readTerrain(v);
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected updateScale (): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    /**
     * The heightfield is centred on its local origin, whereas a Cocos terrain is anchored
     * at its corner, so the collider is shifted by half its extent.
     */
    setCenter (v: IVec3Like): void {
        Vec3.multiply(offset, v as Vec3, this._collider.node.worldScale);
        Vec3.add(offset, offset, this._localOffset);
        this._desc.setTranslation(offset.x, offset.y, offset.z);
        if (this._impl) this._impl.setTranslationWrtParent(offset);
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        const ws = this._collider.node.worldScale;
        return out.set(
            Math.max(1e-5, (this._sizeI - 1) * this._tileSize * 0.5 * Math.abs(ws.x)),
            1e5,
            Math.max(1e-5, (this._sizeJ - 1) * this._tileSize * 0.5 * Math.abs(ws.z)),
        );
    }

    protected getLocalBoundingRadius (): number {
        return this.getLocalHalfExtents(offset).length();
    }

    private _readTerrain (asset: ITerrainAsset | null): void {
        if (!asset) {
            this._sizeI = 0;
            this._sizeJ = 0;
            this.heights = new Float32Array(0);
            return;
        }
        this._tileSize = asset.tileSize;
        this._sizeI = asset.getVertexCountI();
        this._sizeJ = asset.getVertexCountJ();

        // Rapier's `nrows`/`ncols` are CELL counts, so the buffer length is
        // (nrows + 1) * (ncols + 1) == sizeI * sizeJ, and it is COLUMN-major:
        // index = column * (nrows + 1) + row, i.e. j * sizeI + i.
        const heights = new Float32Array(this._sizeI * this._sizeJ);
        for (let j = 0; j < this._sizeJ; j++) {
            for (let i = 0; i < this._sizeI; i++) {
                heights[j * this._sizeI + i] = asset.getHeight(i, j);
            }
        }
        this.heights = heights;
    }

    private _buildDesc (): RAPIER.ColliderDesc {
        if (this._sizeI < 2 || this._sizeJ < 2) {
            this._localOffset.set(0, 0, 0);
            return this.rapier.ColliderDesc.cuboid(1e-5, 1e-5, 1e-5);
        }
        const ws = this._collider.node.worldScale;
        const extentX = (this._sizeI - 1) * this._tileSize * Math.abs(ws.x);
        const extentZ = (this._sizeJ - 1) * this._tileSize * Math.abs(ws.z);
        this._localOffset.set(extentX * 0.5, 0, extentZ * 0.5);

        return this.rapier.ColliderDesc.heightfield(
            this._sizeI - 1,
            this._sizeJ - 1,
            this.heights,
            { x: extentX, y: Math.abs(ws.y), z: extentZ },
            ERapierHeightFieldFlags.FIX_INTERNAL_EDGES as RAPIER.HeightFieldFlags,
        );
    }
}
```

- [ ] **Step 5: Write the simplex shape**

Create `cocos/physics/rapier/shapes/rapier-simplex-shape.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3, warn } from '../../../core';
import { SimplexCollider } from '../../../../exports/physics-framework';
import { ISimplexShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';

const scratch = new Vec3();

/** @mangle */
export class RapierSimplexShape extends RapierShape implements ISimplexShape {
    private _points = new Float32Array(0);

    get collider (): SimplexCollider {
        return this._collider as SimplexCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._desc = this._buildDesc();
    }

    setShapeType (_v: SimplexCollider.ESimplexType): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    setVertices (_v: IVec3Like[]): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected updateScale (): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        let x = 1e-5;
        let y = 1e-5;
        let z = 1e-5;
        for (let i = 0; i < this._points.length; i += 3) {
            x = Math.max(x, Math.abs(this._points[i]));
            y = Math.max(y, Math.abs(this._points[i + 1]));
            z = Math.max(z, Math.abs(this._points[i + 2]));
        }
        return out.set(x, y, z);
    }

    protected getLocalBoundingRadius (): number {
        return this.getLocalHalfExtents(scratch).length();
    }

    /**
     * `ESimplexType` doubles as the vertex count (POINT = 1 ... TETRAHEDRON = 4), the same
     * trick the bullet backend uses. A convex hull needs at least 4 non-coplanar points, so
     * lower counts fall back to a small ball rather than failing.
     */
    private _buildDesc (): RAPIER.ColliderDesc {
        const collider = this.collider;
        const count = collider.shapeType as number;
        const vertices = collider.vertices;
        const ws = collider.node.worldScale;

        const points = new Float32Array(count * 3);
        for (let i = 0; i < count; i++) {
            points[i * 3] = vertices[i].x * ws.x;
            points[i * 3 + 1] = vertices[i].y * ws.y;
            points[i * 3 + 2] = vertices[i].z * ws.z;
        }
        this._points = points;

        if (count >= 4) {
            const hull = this.rapier.ColliderDesc.convexHull(points);
            if (hull) return hull;
            warn(`[PHYSICS][rapier]: degenerate simplex on '${collider.node.name}', falling back to a sphere.`);
        }
        return this.rapier.ColliderDesc.ball(Math.max(1e-5, this.getLocalBoundingRadius()));
    }
}
```

- [ ] **Step 6: Register the wrappers**

In `cocos/physics/rapier/instantiate.ts`:

```ts
import { RapierTerrainShape } from './shapes/rapier-terrain-shape';
import { RapierSimplexShape } from './shapes/rapier-simplex-shape';
```

```ts
        TerrainShape: RapierTerrainShape,
        SimplexShape: RapierSimplexShape,
```

Delete the "deliberately left unregistered" comment block; every slot is now filled.

- [ ] **Step 7: Run the tests to verify they pass**

Run: `npx jest tests/physics/rapier-internals.test.ts && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 8: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier terrain and simplex shapes"
```

---

## Task 9: Debug draw

`world.debugRender()` returns the entire scene as one line-list, making this the cheapest debug draw of any backend.

**Files:**
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: `RapierWorld._debugDrawFlags` from Task 0.
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier internals', ...)`:

```ts
    test('debugRender produces line data when WIRE_FRAME is set', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const world = PhysicsSystem.instance.physicsWorld;
        world.debugDrawFlags = physics.EPhysicsDrawFlags.WIRE_FRAME;
        const buffers = (world as any).impl.debugRender();
        expect(buffers.vertices.length).toBeGreaterThan(0);
        expect(buffers.colors.length).toBeGreaterThan(0);

        // Must not throw with no camera available in the test environment.
        expect(() => { (world as any)._debugDraw(); }).not.toThrow();
    });
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "debugRender"`
Expected: FAIL with `TypeError: world._debugDraw is not a function`.

- [ ] **Step 3: Implement debug draw**

In `cocos/physics/rapier/rapier-world.ts`, add imports and state:

```ts
import { Color, director, geometry } from '../../core';
import { GeometryRenderer } from '../../rendering/geometry-renderer';
```

```ts
    private readonly _MAX_DEBUG_LINE_COUNT = 16384;
    private _debugLineCount = 0;
    private readonly _aabbColor = new Color(0, 255, 255, 255);
    private readonly _wireColor = new Color(255, 255, 255, 255);
    private readonly _debugV3_0 = new Vec3();
    private readonly _debugV3_1 = new Vec3();
    private readonly _debugAABB = new geometry.AABB();
```

Call it at the tail of `step`, after the force reset loop:

```ts
        this._debugDraw();
```

```ts
    /** Same accessor every other backend uses; returns null when there is no camera. */
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
            // Rapier renders the whole world in one call: a flat line list of 3 floats per
            // vertex, 2 vertices per line, with one RGBA colour per vertex.
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
                    this._wireColor.set(colors[c] * 255, colors[c + 1] * 255, colors[c + 2] * 255, colors[c + 3] * 255);
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
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "debugRender" && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 5: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier debug draw via world.debugRender"
```

---

## Task 10: Editor feature entry

Makes the backend selectable in Cocos Creator's UI rather than only from code.

**Files:**
- Modify: `editor/engine-features/render-config.json`
- Modify: `editor/i18n/en/localization.js`
- Modify: `editor/i18n/zh/localization.js`
- Test: `tests/physics/rapier-internals.test.ts`

**Interfaces:**
- Consumes: the `physics-rapier` feature and `LOAD_RAPIER_MANUALLY` constant already in `cc.config.json`.
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Write the failing test**

Append to `tests/physics/rapier-internals.test.ts`, outside the `describe` block:

```ts
test('editor exposes the rapier physics feature with matching i18n keys', () => {
    const config = require('../../editor/engine-features/render-config.json');
    const en = require('../../editor/i18n/en/localization.js');
    const zh = require('../../editor/i18n/zh/localization.js');

    const option = config.physics.options['physics-rapier'];
    expect(option).toBeDefined();
    // Web-only: must not be flagged as a native module.
    expect(option.isNativeModule).toBeUndefined();
    expect(option.cmakeConfig).toBeUndefined();
    expect(option.default).toBeUndefined();
    expect(option.flags.LOAD_RAPIER_MANUALLY).toBeDefined();

    for (const bundle of [en, zh]) {
        const features = bundle.ENGINE ? bundle.ENGINE.features : bundle.features;
        expect(features.physics_rapier.label).toBeTruthy();
        expect(features.flags.rapier.loadManual.label).toBeTruthy();
    }
});
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "editor exposes"`
Expected: FAIL — `option` is `undefined`.

- [ ] **Step 3: Add the feature option**

In `editor/engine-features/render-config.json`, insert after the `physics-physx` block inside `physics.options`:

```json
                "physics-rapier": {
                    "label": "i18n:ENGINE.features.physics_rapier.label",
                    "description": "i18n:ENGINE.features.physics_rapier.description",
                    "flags": {
                        "LOAD_RAPIER_MANUALLY": {
                            "label": "i18n:ENGINE.features.flags.rapier.loadManual.label",
                            "description": "i18n:ENGINE.features.flags.rapier.loadManual.description",
                            "default": false,
                            "ui-type": "checkbox"
                        }
                    }
                },
```

- [ ] **Step 4: Add the localisation strings**

In `editor/i18n/en/localization.js`, after the `physics_physx` block:

```js
        physics_rapier: {
            label: "Rapier Based Physics System",
            description: "Physics system that based on Rapier. Web only, WebAssembly required.",
        },
```

and inside `features.flags`, after the `physx` entry:

```js
            rapier: {
                loadManual: {
                    label: 'Load Manually',
                    description: `Whether to load Rapier Wasm moudle manually by 'loadWasmModuleRapier' API ?`,
                },
            },
```

(The misspelling `moudle` is deliberate — it matches the four sibling entries.)

In `editor/i18n/zh/localization.js`, after its `physics_physx` block:

```js
        physics_rapier: {
            label: "基于 Rapier 的物理系统",
            description: "基于 Rapier 的物理系统支持。仅支持 Web 平台，需要 WebAssembly。",
        },
```

and inside its `features.flags`, after `physx`:

```js
            rapier: {
                loadManual: {
                    label: '手动加载',
                    description: `是否通过 'loadWasmModuleRapier' API 手动加载 Rapier Wasm 模块 ?`,
                },
            },
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-internals.test.ts -t "editor exposes"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add editor tests/physics
git commit -m "feat(editor): expose the rapier physics backend as a build feature"
```

---

## Task 11: Extras — typed access, enums, and tuning

The spec is a lowest common denominator across four engines. Everything Rapier-only lives in `extras/`, a side-effect-free barrel so unreferenced parts tree-shake away.

**Files:**
- Create: `cocos/physics/rapier/extras/rapier-access.ts`
- Create: `cocos/physics/rapier/extras/rapier-enums.ts`
- Create: `cocos/physics/rapier/extras/rapier-solver-config.ts`
- Create: `cocos/physics/rapier/extras/rapier-body-tuning.ts`
- Create: `cocos/physics/rapier/extras/rapier-collider-tuning.ts`
- Create: `cocos/physics/rapier/extras/index.ts`
- Modify: `exports/physics-rapier.ts`
- Test: `tests/physics/rapier-extras.test.ts` (create)

**Interfaces:**
- Consumes: `R`, `isRapierReady`, `RapierWorld`, `RapierRigidBody`, `RapierShape`.
- Produces: `getRapier()`, `getRapierWorld()`, `getRapierRigidBody(component)`, `getRapierCollider(component)`, `isRapierActive()`, `rapierSolver`, and the `setRapier*` tuning functions used by Task 13.

- [ ] **Step 1: Write the failing test**

Create `tests/physics/rapier-extras.test.ts`:

```ts
import { director, game, Game } from "../../cocos/game";
import { physics, PhysicsSystem } from "../../exports/physics-framework";
import "../../exports/physics-rapier";
import "../../exports/physics-cannon";
import { waitForRapierInstantiation } from "../../cocos/physics/rapier/instantiated";
import {
    getRapier, getRapierWorld, getRapierRigidBody, getRapierCollider, isRapierActive,
    rapierSolver, setRapierCcdEnabled, setRapierDominanceGroup, setRapierContactSkin,
    ERapierActiveEvents, ERapierQueryFilterFlags,
} from "../../cocos/physics/rapier/extras";
import { Node, Scene } from "../../cocos/scene-graph";

beforeAll(async () => {
    await waitForRapierInstantiation();
    game.emit(Game.EVENT_PRE_SUBSYSTEM_INIT);
    PhysicsSystem.constructAndRegister();
});

describe('rapier extras', () => {
    let scene: Scene;

    beforeEach(() => {
        physics.selector.switchTo('rapier');
        scene = new Scene('rapier-extras');
        director.runSceneImmediate(scene);
    });

    afterEach(() => { scene.destroy(); });

    test('mirrored enums match the real Rapier values', () => {
        const R = getRapier()!;
        expect(ERapierActiveEvents.COLLISION_EVENTS).toBe(R.ActiveEvents.COLLISION_EVENTS);
        expect(ERapierQueryFilterFlags.EXCLUDE_SENSORS).toBe(R.QueryFilterFlags.EXCLUDE_SENSORS);
    });

    test('typed accessors resolve the live objects', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        expect(isRapierActive()).toBe(true);
        expect(getRapierWorld()).toBe((PhysicsSystem.instance.physicsWorld as any).impl);
        expect(getRapierRigidBody(rb)).not.toBeNull();
        expect(getRapierCollider(collider)).not.toBeNull();
    });

    test('accessors return null when rapier is not active', () => {
        physics.selector.switchTo('cannon.js');
        expect(isRapierActive()).toBe(false);
        expect(getRapierWorld()).toBeNull();
    });

    test('solver settings round-trip', () => {
        rapierSolver.numSolverIterations = 8;
        expect(rapierSolver.numSolverIterations).toBe(8);
        rapierSolver.lengthUnit = 2;
        expect(rapierSolver.lengthUnit).toBeCloseTo(2, 5);
    });

    test('per-body and per-collider tuning reach Rapier', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.ERigidBodyType.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        setRapierCcdEnabled(rb, true);
        expect(getRapierRigidBody(rb)!.isCcdEnabled()).toBe(true);

        setRapierDominanceGroup(rb, 5);
        expect(getRapierRigidBody(rb)!.dominanceGroup()).toBe(5);

        setRapierContactSkin(collider, 0.05);
        expect(getRapierCollider(collider)!.contactSkin()).toBeCloseTo(0.05, 5);
    });
});
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `npx jest tests/physics/rapier-extras.test.ts`
Expected: FAIL — `Cannot find module '../../cocos/physics/rapier/extras'`.

- [ ] **Step 3: Write the typed accessors**

Create `cocos/physics/rapier/extras/rapier-access.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Collider, PhysicsSystem, RigidBody, CharacterController } from '../../../../exports/physics-framework';
import { selector } from '../../framework/physics-selector';
import { R, isRapierReady } from '../instantiated';
import type { RapierWorld } from '../rapier-world';
import type { RapierRigidBody } from '../rapier-rigid-body';
import type { RapierShape } from '../shapes/rapier-shape';
import type { RapierCharacterController } from '../character-controllers/rapier-character-controller';

/**
 * Every accessor returns null when rapier is not the active backend, so calling code can
 * feature-detect rather than hand-casting the untyped `impl` escape hatch.
 */

export function isRapierActive (): boolean {
    return selector.id === 'rapier' && isRapierReady();
}

/** The whole RAPIER namespace: enums, ColliderDesc, JointData, and everything unwrapped. */
export function getRapier (): typeof RAPIER | null {
    return isRapierReady() ? R : null;
}

export function getRapierWorld (): RAPIER.World | null {
    if (!isRapierActive()) return null;
    const world = PhysicsSystem.instance.physicsWorld as RapierWorld | null;
    return world ? world.impl : null;
}

export function getRapierRigidBody (body: RigidBody): RAPIER.RigidBody | null {
    if (!isRapierActive() || !body.body) return null;
    return (body.body as RapierRigidBody).sharedBody.impl;
}

export function getRapierCollider (collider: Collider): RAPIER.Collider | null {
    if (!isRapierActive() || !collider.shape) return null;
    return (collider.shape as unknown as RapierShape).impl;
}

export function getRapierCharacterController (cct: CharacterController): RAPIER.KinematicCharacterController | null {
    if (!isRapierActive()) return null;
    const impl = (cct as unknown as { _cct: RapierCharacterController | null })._cct;
    return impl ? impl.impl : null;
}
```

- [ ] **Step 4: Write the enum re-exports and tuning helpers**

Create `cocos/physics/rapier/extras/rapier-enums.ts` (licence header first):

```ts
/**
 * Re-exports the enum mirrors from `rapier-enum.ts` under public names, plus the ones only
 * the extras API needs. These are local copies rather than a static import of the Rapier
 * package, which would pull the 2.8 MB WASM chunk into whatever chunk references them.
 * `tests/physics/rapier-extras.test.ts` asserts them against the real values after init.
 */
export {
    ERapierActiveEvents,
    ERapierActiveCollisionTypes,
    ERapierQueryFilterFlags,
    ERapierCoefficientCombineRule,
    ERapierJointAxesMask,
} from '../rapier-enum';

/** Mirror of `RAPIER.ActiveHooks`. Note the plural member names. */
export const ERapierActiveHooks = {
    NONE: 0,
    FILTER_CONTACT_PAIRS: 1,
    FILTER_INTERSECTION_PAIRS: 2,
} as const;

/** Mirror of `RAPIER.SolverFlags`. */
export const ERapierSolverFlags = {
    EMPTY: 0,
    COMPUTE_IMPULSE: 1,
} as const;

/** Mirror of `RAPIER.MotorModel`. */
export const ERapierMotorModel = {
    ACCELERATION_BASED: 0,
    FORCE_BASED: 1,
} as const;

/** Mirror of `RAPIER.JointAxis`. */
export const ERapierJointAxis = {
    LIN_X: 0,
    LIN_Y: 1,
    LIN_Z: 2,
    ANG_X: 3,
    ANG_Y: 4,
    ANG_Z: 5,
} as const;
```

Create `cocos/physics/rapier/extras/rapier-solver-config.ts` (licence header first):

```ts
import { getRapierWorld } from './rapier-access';

/**
 * World-level solver tuning with no Cocos analogue.
 *
 * These write live `RAPIER.World` state, so they must be re-applied after
 * `selector.switchTo()` rebuilds the world.
 *
 * Note two asymmetries inherited from Rapier 0.20: `contactErp` is read-only, and
 * `contactNaturalFrequency` is write-only. The old `erp` property no longer exists.
 */
export const rapierSolver = {
    get numSolverIterations (): number {
        return getRapierWorld()?.numSolverIterations ?? 0;
    },
    set numSolverIterations (v: number) {
        const w = getRapierWorld();
        if (w) w.numSolverIterations = v;
    },

    get numInternalPgsIterations (): number {
        return getRapierWorld()?.numInternalPgsIterations ?? 0;
    },
    set numInternalPgsIterations (v: number) {
        const w = getRapierWorld();
        if (w) w.numInternalPgsIterations = v;
    },

    get maxCcdSubsteps (): number {
        return getRapierWorld()?.maxCcdSubsteps ?? 0;
    },
    set maxCcdSubsteps (v: number) {
        const w = getRapierWorld();
        if (w) w.maxCcdSubsteps = v;
    },

    /** World units per metre. Rapier is tuned for 1 unit = 1 metre. */
    get lengthUnit (): number {
        return getRapierWorld()?.lengthUnit ?? 1;
    },
    set lengthUnit (v: number) {
        const w = getRapierWorld();
        if (w) w.lengthUnit = v;
    },

    get normalizedAllowedLinearError (): number {
        return getRapierWorld()?.integrationParameters.normalizedAllowedLinearError ?? 0;
    },
    set normalizedAllowedLinearError (v: number) {
        const w = getRapierWorld();
        if (w) w.integrationParameters.normalizedAllowedLinearError = v;
    },

    /** Read-only in Rapier 0.20; set `contactNaturalFrequency` instead. */
    get contactErp (): number {
        return getRapierWorld()?.integrationParameters.contact_erp ?? 0;
    },

    /** Write-only in Rapier 0.20; there is no matching getter. */
    set contactNaturalFrequency (v: number) {
        const w = getRapierWorld();
        if (w) w.integrationParameters.contact_natural_frequency = v;
    },
};
```

Create `cocos/physics/rapier/extras/rapier-body-tuning.ts` (licence header first):

```ts
import { RigidBody } from '../../../../exports/physics-framework';
import { getRapierRigidBody } from './rapier-access';

/**
 * Per-body capabilities `IRigidBody` cannot express. Free functions rather than a wrapper
 * class so each one tree-shakes individually.
 */

export function setRapierCcdEnabled (body: RigidBody, enabled: boolean): void {
    getRapierRigidBody(body)?.enableCcd(enabled);
}

/**
 * Soft CCD is a cheaper alternative to full CCD: it prevents tunnelling for slow-but-thin
 * and moderately fast bodies. 0 disables it.
 */
export function setRapierSoftCcdPrediction (body: RigidBody, distance: number): void {
    getRapierRigidBody(body)?.setSoftCcdPrediction(distance);
}

/**
 * A body with a higher dominance group is treated as infinite-mass by lower ones, so it
 * pushes them but is never pushed back. Range is [-127, 127].
 */
export function setRapierDominanceGroup (body: RigidBody, group: number): void {
    getRapierRigidBody(body)?.setDominanceGroup(group);
}

/** Extra solver iterations for this body alone, e.g. one ragdoll without a global cost. */
export function setRapierAdditionalSolverIterations (body: RigidBody, iterations: number): void {
    getRapierRigidBody(body)?.setAdditionalSolverIterations(iterations);
}
```

Create `cocos/physics/rapier/extras/rapier-collider-tuning.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Collider } from '../../../../exports/physics-framework';
import { getRapierCollider } from './rapier-access';

/** Per-collider capabilities `IBaseShape` cannot express. */

/**
 * A speculative contact margin. A small skin markedly improves stacking stability without
 * visibly separating bodies.
 */
export function setRapierContactSkin (collider: Collider, thickness: number): void {
    getRapierCollider(collider)?.setContactSkin(thickness);
}

/**
 * Solver groups are separate from collision groups: a filtered pair still generates contact
 * events but produces no forces. Cocos group/mask maps onto collision groups only.
 */
export function setRapierSolverGroups (collider: Collider, groups: number): void {
    getRapierCollider(collider)?.setSolverGroups(groups);
}

export function setRapierActiveHooks (collider: Collider, hooks: number): void {
    getRapierCollider(collider)?.setActiveHooks(hooks as RAPIER.ActiveHooks);
}

export function setRapierActiveCollisionTypes (collider: Collider, types: number): void {
    getRapierCollider(collider)?.setActiveCollisionTypes(types as RAPIER.ActiveCollisionTypes);
}

/** Minimum total force before a contact-force event fires for this collider. */
export function setRapierContactForceEventThreshold (collider: Collider, threshold: number): void {
    getRapierCollider(collider)?.setContactForceEventThreshold(threshold);
}

export function setRapierFrictionCombineRule (collider: Collider, rule: number): void {
    getRapierCollider(collider)?.setFrictionCombineRule(rule as RAPIER.CoefficientCombineRule);
}

export function setRapierRestitutionCombineRule (collider: Collider, rule: number): void {
    getRapierCollider(collider)?.setRestitutionCombineRule(rule as RAPIER.CoefficientCombineRule);
}
```

Create `cocos/physics/rapier/extras/index.ts` (licence header first):

```ts
// Pure re-export barrel; no top-level statements, so unreferenced exports tree-shake away.
export * from './rapier-access';
export * from './rapier-enums';
export * from './rapier-solver-config';
export * from './rapier-body-tuning';
export * from './rapier-collider-tuning';
```

- [ ] **Step 5: Re-export from the module entry point**

In `exports/physics-rapier.ts`, append:

```ts
// Rapier-only extension surface. Tree-shakeable: the barrel has no side effects.
export * from '../cocos/physics/rapier/extras';
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `npx jest tests/physics/rapier-extras.test.ts && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 7: Commit**

```bash
git add cocos/physics/rapier exports tests/physics
git commit -m "feat(physics): add rapier extras access, enums and tuning helpers"
```

---

## Task 12: Extras — physics hooks, contact-force events, queries

**Files:**
- Create: `cocos/physics/rapier/extras/rapier-hooks.ts`
- Create: `cocos/physics/rapier/extras/rapier-events.ts`
- Create: `cocos/physics/rapier/extras/rapier-queries.ts`
- Modify: `cocos/physics/rapier/extras/index.ts`
- Modify: `cocos/physics/rapier/rapier-world.ts`
- Test: `tests/physics/rapier-extras.test.ts`

**Interfaces:**
- Consumes: `getRapierWorld`, `RapierCache.getShape`, `ERapierSolverFlags`.
- Produces: `setRapierPhysicsHooks(hooks | null)`, `rapierContactForceEvents.on/off`, `rapierProjectPoint`, `rapierCollidersInAabb`. `RapierWorld.setPhysicsHooks`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier extras', ...)`:

```ts
    test('physics hooks can veto a contact pair', () => {
        const { setRapierPhysicsHooks, setRapierActiveHooks, ERapierActiveHooks } =
            require('../../cocos/physics/rapier/extras');

        const floor = new Node('floor');
        scene.addChild(floor);
        floor.worldPosition = new Vec3(0, 0, 0);
        const floorCollider = floor.addComponent(physics.BoxCollider) as physics.BoxCollider;
        floorCollider.size = new Vec3(10, 1, 10);

        const ball = new Node('ball');
        scene.addChild(ball);
        ball.worldPosition = new Vec3(0, 5, 0);
        const ballCollider = ball.addComponent(physics.SphereCollider) as physics.SphereCollider;
        const rb = ball.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.ERigidBodyType.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        setRapierActiveHooks(ballCollider, ERapierActiveHooks.FILTER_CONTACT_PAIRS);
        let called = false;
        setRapierPhysicsHooks({
            filterContactPair: () => { called = true; return null; },
        });

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 120; i++) director.tick(dt);

        expect(called).toBe(true);
        // Contacts were vetoed, so the ball falls straight through the floor.
        expect(ball.worldPosition.y).toBeLessThan(0);
        setRapierPhysicsHooks(null);
    });

    test('point projection finds the nearest collider', () => {
        const { rapierProjectPoint } = require('../../cocos/physics/rapier/extras');
        const node = new Node('box');
        scene.addChild(node);
        node.worldPosition = new Vec3(0, 0, 0);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const hit = rapierProjectPoint(new Vec3(5, 0, 0), true);
        expect(hit).not.toBeNull();
        expect(hit.collider.node.name).toBe('box');
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics/rapier-extras.test.ts -t "hooks"`
Expected: FAIL — `setRapierPhysicsHooks is not a function`.

- [ ] **Step 3: Let the world carry hooks**

In `cocos/physics/rapier/rapier-world.ts`:

```ts
    private _hooks: RAPIER.PhysicsHooks | null = null;

    /**
     * Installs per-step physics hooks. Only colliders that also opt in through
     * `setActiveHooks` will reach the callbacks.
     */
    setPhysicsHooks (hooks: RAPIER.PhysicsHooks | null): void {
        this._hooks = hooks;
    }
```

Pass them to the step:

```ts
        this._world.step(this._eventQueue, this._hooks ?? undefined);
```

- [ ] **Step 4: Write the hooks adapter**

Create `cocos/physics/rapier/extras/rapier-hooks.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Collider, PhysicsSystem } from '../../../../exports/physics-framework';
import { RapierCache } from '../rapier-cache';
import type { RapierWorld } from '../rapier-world';
import { ERapierSolverFlags } from './rapier-enums';
import { isRapierActive } from './rapier-access';

/**
 * Rapier hands the callbacks raw integer handles; this adapter resolves them to Cocos
 * components. Both members are optional here even though Rapier's own interface requires
 * both — the adapter always supplies a default for whichever is missing.
 */
export interface IRapierPhysicsHooks {
    /** Return `ERapierSolverFlags.COMPUTE_IMPULSE` to keep the contact, or null to veto it. */
    filterContactPair? (a: Collider | null, b: Collider | null): number | null;
    /** Return false to veto an intersection (sensor) pair. */
    filterIntersectionPair? (a: Collider | null, b: Collider | null): boolean;
}

function resolve (handle: number): Collider | null {
    const shape = RapierCache.getShape(handle);
    return shape ? shape.collider : null;
}

/**
 * Installs hooks for every subsequent step, or clears them with null.
 *
 * Cost warning: these callbacks cross the JS/WASM boundary for every candidate pair, every
 * step. Restrict them to the few colliders that need it via `setRapierActiveHooks`; a hook
 * installed without that flag silently never fires.
 */
export function setRapierPhysicsHooks (hooks: IRapierPhysicsHooks | null): void {
    if (!isRapierActive()) return;
    const world = PhysicsSystem.instance.physicsWorld as RapierWorld;
    if (!hooks) {
        world.setPhysicsHooks(null);
        return;
    }
    const adapter: RAPIER.PhysicsHooks = {
        filterContactPair (c1, c2): RAPIER.SolverFlags | null {
            if (!hooks.filterContactPair) return ERapierSolverFlags.COMPUTE_IMPULSE as RAPIER.SolverFlags;
            return hooks.filterContactPair(resolve(c1), resolve(c2)) as RAPIER.SolverFlags | null;
        },
        filterIntersectionPair (c1, c2): boolean {
            if (!hooks.filterIntersectionPair) return true;
            return hooks.filterIntersectionPair(resolve(c1), resolve(c2));
        },
    };
    world.setPhysicsHooks(adapter);
}
```

- [ ] **Step 5: Write the contact-force events and queries**

Create `cocos/physics/rapier/extras/rapier-events.ts` (licence header first):

```ts
import { Collider } from '../../../../exports/physics-framework';
import { Vec3 } from '../../../core';

/**
 * Contact-force events, which `ICollisionEvent` has no equivalent for. Useful for
 * destructible objects and impact audio.
 *
 * The underlying `TempContactForceEvent` is a temporary wasm view, so every field is copied
 * before dispatch and listeners must not retain the event object itself.
 */
export interface IRapierContactForceEvent {
    colliderA: Collider | null;
    colliderB: Collider | null;
    totalForce: Vec3;
    totalForceMagnitude: number;
    maxForceDirection: Vec3;
    maxForceMagnitude: number;
}

export type RapierContactForceCallback = (event: IRapierContactForceEvent) => void;

const listeners: RapierContactForceCallback[] = [];

export const rapierContactForceEvents = {
    on (cb: RapierContactForceCallback): void {
        if (listeners.indexOf(cb) < 0) listeners.push(cb);
    },
    off (cb: RapierContactForceCallback): void {
        const i = listeners.indexOf(cb);
        if (i >= 0) listeners.splice(i, 1);
    },
    /** @internal Called by RapierWorld after each step. */
    _dispatch (event: IRapierContactForceEvent): void {
        for (let i = 0; i < listeners.length; i++) listeners[i](event);
    },
    /** @internal */
    _hasListeners (): boolean {
        return listeners.length > 0;
    },
};
```

Create `cocos/physics/rapier/extras/rapier-queries.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { Collider, RigidBody } from '../../../../exports/physics-framework';
import { IVec3Like, Vec3 } from '../../../core';
import { RapierCache } from '../rapier-cache';
import { getRapierWorld, getRapierCollider, getRapierRigidBody } from './rapier-access';

/** Query filtering richer than `IRaycastOptions`. */
export interface IRapierQueryFilter {
    /** A combination of `ERapierQueryFilterFlags`. */
    flags?: number;
    /** A packed Rapier `InteractionGroups` value. */
    groups?: number;
    excludeCollider?: Collider;
    excludeRigidBody?: RigidBody;
    predicate?: (collider: Collider | null) => boolean;
}

export interface IRapierPointProjection {
    collider: Collider;
    point: Vec3;
    isInside: boolean;
}

function wrapPredicate (filter?: IRapierQueryFilter): ((c: RAPIER.Collider) => boolean) | undefined {
    if (!filter?.predicate) return undefined;
    const predicate = filter.predicate;
    return (c: RAPIER.Collider): boolean => {
        const shape = RapierCache.getShape(c.handle);
        return predicate(shape ? shape.collider : null);
    };
}

/** Finds the closest point on any collider to `point`. No Cocos spec equivalent. */
export function rapierProjectPoint (
    point: IVec3Like,
    solid: boolean,
    filter?: IRapierQueryFilter,
): IRapierPointProjection | null {
    const world = getRapierWorld();
    if (!world) return null;
    const hit = world.projectPoint(
        point as RAPIER.Vector,
        solid,
        filter?.flags as RAPIER.QueryFilterFlags | undefined,
        filter?.groups,
        filter?.excludeCollider ? getRapierCollider(filter.excludeCollider) ?? undefined : undefined,
        filter?.excludeRigidBody ? getRapierRigidBody(filter.excludeRigidBody) ?? undefined : undefined,
        wrapPredicate(filter),
    );
    if (!hit) return null;
    const shape = RapierCache.getShape(hit.collider.handle);
    if (!shape || !shape.collider) return null;
    return {
        collider: shape.collider,
        point: new Vec3(hit.point.x, hit.point.y, hit.point.z),
        isInside: hit.isInside,
    };
}

/** Enumerates colliders whose AABB overlaps the given box. Return false to stop early. */
export function rapierCollidersInAabb (
    center: IVec3Like,
    halfExtents: IVec3Like,
    callback: (collider: Collider) => boolean,
): void {
    const world = getRapierWorld();
    if (!world) return;
    world.collidersWithAabbIntersectingAabb(
        center as RAPIER.Vector,
        halfExtents as RAPIER.Vector,
        (c): boolean => {
            const shape = RapierCache.getShape(c.handle);
            if (!shape || !shape.collider) return true;
            return callback(shape.collider);
        },
    );
}
```

- [ ] **Step 6: Drain contact-force events in the world**

In `cocos/physics/rapier/rapier-world.ts`, inside `_emitCCTEvents`'s caller `emitEvents`, after `_emitCCTEvents()`:

```ts
        this._emitContactForceEvents();
```

```ts
    private readonly _forceEvent = {
        colliderA: null as Collider | null,
        colliderB: null as Collider | null,
        totalForce: new Vec3(),
        totalForceMagnitude: 0,
        maxForceDirection: new Vec3(),
        maxForceMagnitude: 0,
    };

    private _emitContactForceEvents (): void {
        if (!rapierContactForceEvents._hasListeners()) return;
        this._eventQueue.drainContactForceEvents((event): void => {
            const a = RapierCache.getShape(event.collider1());
            const b = RapierCache.getShape(event.collider2());
            const e = this._forceEvent;
            e.colliderA = a ? a.collider : null;
            e.colliderB = b ? b.collider : null;
            // Copy before the callback returns; the event view is freed afterwards.
            event.totalForce(e.totalForce);
            e.totalForceMagnitude = event.totalForceMagnitude();
            event.maxForceDirection(e.maxForceDirection);
            e.maxForceMagnitude = event.maxForceMagnitude();
            rapierContactForceEvents._dispatch(e);
        });
    }
```

Import `rapierContactForceEvents` from `./extras/rapier-events` and `Collider` from the framework exports.

- [ ] **Step 7: Extend the barrel**

In `cocos/physics/rapier/extras/index.ts`:

```ts
export * from './rapier-hooks';
export * from './rapier-events';
export * from './rapier-queries';
```

- [ ] **Step 8: Run the tests to verify they pass**

Run: `npx jest tests/physics/rapier-extras.test.ts && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 9: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier hooks, contact-force events and extra queries"
```

---

## Task 13: Extras — debug buffers, snapshots, joints, character tuning

**Files:**
- Create: `cocos/physics/rapier/extras/rapier-debug-render.ts`
- Create: `cocos/physics/rapier/extras/rapier-snapshot.ts`
- Create: `cocos/physics/rapier/extras/rapier-joints.ts`
- Create: `cocos/physics/rapier/extras/rapier-character.ts`
- Modify: `cocos/physics/rapier/extras/index.ts`
- Test: `tests/physics/rapier-extras.test.ts`

**Interfaces:**
- Consumes: `getRapier`, `getRapierWorld`, `getRapierRigidBody`, `getRapierCharacterController`.
- Produces: `rapierDebugRenderBuffers`, `rapierTakeSnapshot`, `createRapierSpringJoint`, `createRapierRopeJoint`, `destroyRapierJoint`, `getRapierCharacterTuning`.

- [ ] **Step 1: Write the failing test**

Append to `describe('rapier extras', ...)`:

```ts
    test('spring joint is created and destroyed cleanly', () => {
        const { createRapierSpringJoint, destroyRapierJoint } =
            require('../../cocos/physics/rapier/extras');

        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.ERigidBodyType.DYNAMIC;
        rbB.type = physics.ERigidBodyType.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const world = (PhysicsSystem.instance.physicsWorld as any).impl;
        const before = world.impulseJoints.len();
        const handle = createRapierSpringJoint(rbA, rbB, {
            restLength: 2, stiffness: 10, damping: 1,
        });
        expect(handle).not.toBeNull();
        expect(world.impulseJoints.len()).toBe(before + 1);

        destroyRapierJoint(handle);
        expect(world.impulseJoints.len()).toBe(before);
    });

    test('debug render buffers and snapshots are available', () => {
        const { rapierDebugRenderBuffers, rapierTakeSnapshot } =
            require('../../cocos/physics/rapier/extras');

        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const buffers = rapierDebugRenderBuffers();
        expect(buffers!.vertices.length).toBeGreaterThan(0);

        const snapshot = rapierTakeSnapshot();
        expect(snapshot!.byteLength).toBeGreaterThan(0);
    });

    test('character tuning exposes autostep and snap-to-ground', () => {
        const { getRapierCharacterTuning } = require('../../cocos/physics/rapier/extras');
        const node = new Node('cct');
        scene.addChild(node);
        const cct = node.addComponent(physics.CapsuleCharacterController) as physics.CapsuleCharacterController;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const tuning = getRapierCharacterTuning(cct);
        expect(tuning).not.toBeNull();
        tuning.snapToGroundDistance = 0.3;
        expect(tuning.snapToGroundDistance).toBeCloseTo(0.3, 5);
        tuning.slideEnabled = false;
        expect(tuning.slideEnabled).toBe(false);
    });
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `npx jest tests/physics/rapier-extras.test.ts -t "spring joint"`
Expected: FAIL — `createRapierSpringJoint is not a function`.

- [ ] **Step 3: Write debug buffers and snapshots**

Create `cocos/physics/rapier/extras/rapier-debug-render.ts` (licence header first):

```ts
import { getRapierWorld } from './rapier-access';

/**
 * Raw debug geometry for the whole world, as a flat line list: 3 floats per vertex,
 * 2 vertices per line, with one RGBA colour per vertex.
 *
 * The arrays are the pipeline's own buffers, re-wrapped on each call, so copy them if you
 * need to retain them across steps.
 */
export function rapierDebugRenderBuffers (): { vertices: Float32Array; colors: Float32Array } | null {
    const world = getRapierWorld();
    if (!world) return null;
    const buffers = world.debugRender();
    return { vertices: buffers.vertices, colors: buffers.colors };
}
```

Create `cocos/physics/rapier/extras/rapier-snapshot.ts` (licence header first):

```ts
import { warn } from '../../../core';
import { getRapierWorld } from './rapier-access';

/**
 * EXPERIMENTAL. Serializes the whole world, which is the basis for rollback netcode and
 * deterministic replay.
 *
 * Restoring is deliberately NOT provided as an in-place operation: Rapier's
 * `World.restoreSnapshot` is a static that returns a brand-new `World`, which invalidates
 * every collider and rigid-body handle the Cocos wrappers hold. Rebinding all of them
 * safely is a larger change than this accessor, so restoring is exposed only as the raw
 * world, leaving the caller responsible for the consequences.
 */
export function rapierTakeSnapshot (): Uint8Array | null {
    const world = getRapierWorld();
    return world ? world.takeSnapshot() : null;
}

/**
 * Returns a NEW detached `RAPIER.World` built from `data`. The engine's wrappers continue
 * to point at the old world, so this is only useful for inspection or for callers driving
 * Rapier directly.
 */
export function rapierWorldFromSnapshot (data: Uint8Array): unknown | null {
    const world = getRapierWorld();
    if (!world) return null;
    warn('[PHYSICS][rapier]: rapierWorldFromSnapshot returns a detached world; the engine\'s '
        + 'colliders and bodies still reference the previous one.');
    return (world.constructor as { restoreSnapshot (d: Uint8Array): unknown }).restoreSnapshot(data);
}
```

- [ ] **Step 4: Write the joint helpers**

Create `cocos/physics/rapier/extras/rapier-joints.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { IVec3Like, Vec3 } from '../../../core';
import { RigidBody } from '../../../../exports/physics-framework';
import { getRapier, getRapierWorld, getRapierRigidBody } from './rapier-access';

const ORIGIN: IVec3Like = new Vec3();

/** Opaque handle returned by the joint creators; pass it to `destroyRapierJoint`. */
export interface IRapierJointHandle {
    readonly impl: RAPIER.ImpulseJoint;
}

export interface IRapierSpringOptions {
    restLength: number;
    stiffness: number;
    damping: number;
    anchorA?: IVec3Like;
    anchorB?: IVec3Like;
}

export interface IRapierRopeOptions {
    length: number;
    anchorA?: IVec3Like;
    anchorB?: IVec3Like;
}

function create (data: RAPIER.JointData, a: RigidBody, b: RigidBody): IRapierJointHandle | null {
    const world = getRapierWorld();
    const bodyA = getRapierRigidBody(a);
    const bodyB = getRapierRigidBody(b);
    if (!world || !bodyA || !bodyB) return null;
    return { impl: world.createImpulseJoint(data, bodyA, bodyB, true) };
}

/** A distance spring. No Cocos constraint type corresponds to this. */
export function createRapierSpringJoint (
    a: RigidBody,
    b: RigidBody,
    options: IRapierSpringOptions,
): IRapierJointHandle | null {
    const R = getRapier();
    if (!R) return null;
    const data = R.JointData.spring(
        options.restLength,
        options.stiffness,
        options.damping,
        (options.anchorA ?? ORIGIN) as RAPIER.Vector,
        (options.anchorB ?? ORIGIN) as RAPIER.Vector,
    );
    return create(data, a, b);
}

/** A maximum-distance rope. No Cocos constraint type corresponds to this. */
export function createRapierRopeJoint (
    a: RigidBody,
    b: RigidBody,
    options: IRapierRopeOptions,
): IRapierJointHandle | null {
    const R = getRapier();
    if (!R) return null;
    const data = R.JointData.rope(
        options.length,
        (options.anchorA ?? ORIGIN) as RAPIER.Vector,
        (options.anchorB ?? ORIGIN) as RAPIER.Vector,
    );
    return create(data, a, b);
}

export function destroyRapierJoint (handle: IRapierJointHandle | null): void {
    const world = getRapierWorld();
    if (!world || !handle) return;
    world.removeImpulseJoint(handle.impl, true);
}
```

- [ ] **Step 5: Write the character tuning wrapper**

Create `cocos/physics/rapier/extras/rapier-character.ts` (licence header first):

```ts
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { CharacterController } from '../../../../exports/physics-framework';
import { getRapierCharacterController } from './rapier-access';

/**
 * The ~15 knobs Rapier's kinematic character controller exposes that the Cocos
 * `CharacterController` component does not. A cohesive set on one entity, so this is a
 * wrapper object rather than free functions.
 *
 * Angles are in radians, matching Rapier.
 */
export class RapierCharacterTuning {
    constructor (private readonly _impl: RAPIER.KinematicCharacterController) {}

    /** Null disables autostep. */
    get autostep (): { maxHeight: number; minWidth: number; includeDynamicBodies: boolean } | null {
        if (!this._impl.autostepEnabled()) return null;
        return {
            maxHeight: this._impl.autostepMaxHeight() ?? 0,
            minWidth: this._impl.autostepMinWidth() ?? 0,
            includeDynamicBodies: this._impl.autostepIncludesDynamicBodies() ?? false,
        };
    }

    set autostep (v: { maxHeight: number; minWidth: number; includeDynamicBodies: boolean } | null) {
        if (!v) this._impl.disableAutostep();
        else this._impl.enableAutostep(v.maxHeight, v.minWidth, v.includeDynamicBodies);
    }

    /** Null disables snapping to the ground. */
    get snapToGroundDistance (): number | null {
        return this._impl.snapToGroundEnabled() ? this._impl.snapToGroundDistance() : null;
    }

    set snapToGroundDistance (v: number | null) {
        if (v === null) this._impl.disableSnapToGround();
        else this._impl.enableSnapToGround(v);
    }

    get maxSlopeClimbAngle (): number { return this._impl.maxSlopeClimbAngle(); }
    set maxSlopeClimbAngle (v: number) { this._impl.setMaxSlopeClimbAngle(v); }

    get minSlopeSlideAngle (): number { return this._impl.minSlopeSlideAngle(); }
    set minSlopeSlideAngle (v: number) { this._impl.setMinSlopeSlideAngle(v); }

    get applyImpulsesToDynamicBodies (): boolean { return this._impl.applyImpulsesToDynamicBodies(); }
    set applyImpulsesToDynamicBodies (v: boolean) { this._impl.setApplyImpulsesToDynamicBodies(v); }

    /** Null means the mass is taken from the character's collider. */
    get characterMass (): number | null { return this._impl.characterMass(); }
    set characterMass (v: number | null) { this._impl.setCharacterMass(v); }

    get slideEnabled (): boolean { return this._impl.slideEnabled(); }
    set slideEnabled (v: boolean) { this._impl.setSlideEnabled(v); }

    get normalNudgeFactor (): number { return this._impl.normalNudgeFactor(); }
    set normalNudgeFactor (v: number) { this._impl.setNormalNudgeFactor(v); }

    get offset (): number { return this._impl.offset(); }
    set offset (v: number) { this._impl.setOffset(v); }

    get grounded (): boolean { return this._impl.computedGrounded(); }
}

export function getRapierCharacterTuning (cct: CharacterController): RapierCharacterTuning | null {
    const impl = getRapierCharacterController(cct);
    return impl ? new RapierCharacterTuning(impl) : null;
}
```

- [ ] **Step 6: Extend the barrel**

In `cocos/physics/rapier/extras/index.ts`:

```ts
export * from './rapier-debug-render';
export * from './rapier-snapshot';
export * from './rapier-joints';
export * from './rapier-character';
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `npx jest tests/physics/rapier-extras.test.ts && npx tsc --noEmit`
Expected: PASS, 0 type errors.

- [ ] **Step 8: Commit**

```bash
git add cocos/physics/rapier tests/physics
git commit -m "feat(physics): add rapier debug buffers, snapshots, joints and character tuning"
```

---

## Task 14: Full regression and documentation

**Files:**
- Modify: `tests/physics/physics.test.ts`
- Modify: `cocos/physics/rapier/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: nothing.

- [ ] **Step 1: Remove every remaining rapier guard**

Confirm `tests/physics/physics.test.ts` has no `id === 'rapier'` or `id !== 'rapier'` branch left:

```bash
grep -n "rapier" tests/physics/physics.test.ts
```

Expected: only the two import lines and the `PhysicsTestEnv.backendId` union.

- [ ] **Step 2: Run the full physics matrix**

Run: `npx jest tests/physics`
Expected: all ten suites run against `rapier` alongside the other four backends, 0 failures.

- [ ] **Step 3: Run the whole suite and the static gates**

Run: `npx jest && npx tsc --noEmit && npx eslint "cocos/physics/rapier/**/*.ts" "exports/physics-rapier.ts"`
Expected: 0 failures, 0 type errors, clean lint.

- [ ] **Step 4: Verify an H5 build**

Run: `npm run build:dev`
Expected: completes. Then run `npm run build:min` and confirm from the emitted treemap that Rapier occupies **its own chunk**, not `cc.js` — that is the payoff of the dynamic `import()` in `instantiated.ts`. If it is inlined, something converted the dynamic import into a static one.

- [ ] **Step 5: Update the backend README**

In `cocos/physics/rapier/README.md`:
- Change the status line to note that every wrapper slot is filled and all ten suites pass.
- Move constraints, character controllers, sweeps, terrain/simplex, debug draw and the editor entry out of "Not yet implemented" into "What is implemented".
- Add to "Known limitations": the generic joint supports only free and locked axes, so `ConfigurableConstraint` LIMITED axes behave as FREE and all its drivers are inert; fixed constraints never break; character-controller trigger overlap events are unsupported.
- Replace the "Future work" section with the shipped `extras/` surface, keeping snapshots marked experimental.

- [ ] **Step 6: Commit**

```bash
git add tests/physics cocos/physics/rapier/README.md
git commit -m "test(physics): run every suite against rapier and update backend docs"
```

---

## Self-Review

**Spec coverage.** Every item in the README's "Remaining" list maps to a task: sweeps (1), constraints (2-5), character controllers (6-7), terrain/simplex (8), debug draw (9), editor integration (10), the deep-Rapier API (11-13), suite wiring (14). The six audit defects map to Task 0.

**Placeholders.** None. Every code step carries the literal content to write; `_warnLimits`-style repeated one-line setters are spelled out rather than elided, and no step says "similar to Task N".

**Type consistency.** `RapierConstraint.buildJointData` / `scheduleRebuild` / `flushRebuild` / `createJoint` / `destroyJoint` are declared in Task 2 and used unchanged in Tasks 3-5. `RapierCharacterController.buildColliderDesc` is declared in Task 6 and implemented in both subclasses in the same task; `pendingContacts` / `recycleContacts` are added in Task 7 and consumed by `RapierWorld._emitCCTEvents` in the same task. `RapierWorld.addConstraint/removeConstraint/anchorBody/constraints` (Task 2), `ccts/addCCT/removeCCT/updateNeedEmitCCTEvents` (Task 6) and `setPhysicsHooks` (Task 12) are each defined before first use. `getRapierCharacterController` is defined in Task 11 and consumed in Task 13.

**Review Focus coverage.** Item 1 is tested in Task 1 Step 1 (zero-length sweep direction). Item 2 in Task 2 Step 1 (`rebinding connectedBody does not leak`). Item 3 in Task 6 Step 1 (sub-threshold move and zero timestep). Item 4 in Task 8 Step 1 (non-square 3x5 terrain, height varying only with `j`). Item 5 is covered by Task 0's double-destroy test plus the joints-before-bodies ordering in Task 2 Step 3 and the CCT teardown in Task 6 Step 3.
