# Rapier physics backend

A fifth 3D physics backend for this fork of cocos-engine 3.8.8, backed by
[Rapier](https://rapier.rs) — a Rust physics engine compiled to WebAssembly with official
JS bindings. It sits alongside the four backends the engine already ships: `bullet`,
`physx`, `cannon.js` and `builtin`.

- **Status:** phase 1 (core) complete and passing tests. Constraints, character controllers
  and shape sweeps are not implemented yet.
- **Selector id:** `rapier`
- **Dependency:** `@dimforge/rapier3d-compat`, pinned exactly at `0.20.0`
- **Platforms:** web / H5 only
- **Size:** 18 TypeScript files, ~3,100 lines

---

## 1. Why this exists

Rapier is actively developed and has a modern solver, but the motivation is a set of
capabilities none of the existing backends expose: deterministic cross-platform stepping,
world snapshots (usable for rollback netcode), a tunable solver, user-supplied physics
hooks, and a built-in kinematic character controller.

Scope decisions:

- **Web / H5 only.** No JSB, no CMake, no mini-game subpackaging. Rapier is Rust, so a
  native Cocos binding would be a separate and far larger effort.
- **Additive.** Registers under a new id. Nothing about the other four backends changes.
- **Core first, then expand.**

Adding a backend is cheap because `cocos/physics/framework/physics-selector.ts` is the
entire integration point — a backend is one `selector.register('rapier', {...})` call, and
every wrapper slot is optional. Unregistered slots degrade to a warning plus a no-op stub
rather than a crash; that is a supported state, not a bug. The shipped `builtin` backend
fills only 5 of 17 slots.

### Why `@dimforge/rapier3d-compat`

An ordinary npm dependency, exactly as `@cocos/cannon` already is. The `-compat` build
inlines the WASM as base64 and exposes an async `init()`.

Measured cost: roughly **1.08 MB gzip**, versus ~787 KB for the non-compat package — about
a 38% penalty for inlining.

Alternatives were rejected:

| Option | Why not |
|---|---|
| `@dimforge/rapier3d` (non-compat) | Saves ~297 KB gzip, but the `.wasm` must be emitted, hashed and URL-resolved at build time. The only engine machinery that does that is ccbuild's `externalWasmLoader`, whose culling table is keyed on **emscripten** suffixes (`.asm.js` / `.wasm.js` / `.js.mem`). Rapier is wasm-bindgen with no asm.js twin, so that loader would cull the binary to `export default ''` under an asmjs build. Would require forking `@cocos/ccbuild`. |
| Vendoring into `native/external/` | That tree is pinned by `native/external-config.json` to `cocos/cocos-engine-external @ v3.8.8-2` and replaced wholesale by `npm run update:native-external`. |

The module is loaded through a **dynamic `import()`** behind a mutable `R` binding, so the
payload lands in its own rollup chunk. Combined with `LOAD_RAPIER_MANUALLY`, a project can
defer the download entirely.

---

## 2. Test results

All commands run from the repo root.

### Rapier backend suites

```
npx jest tests/physics/physics.test.ts -t "Backend: rapier"
→ 11 passed, 0 failed
```

| Test | What it actually validates |
|---|---|
| Event | Trigger and collision `enter` / **`stay`** / `exit`. The `stay` phase is synthesised by this backend (Rapier reports only enter/exit), so this covers the pair-dictionary bookkeeping and the contact-orientation logic. |
| Raycast | `raycast` (all hits) and `raycastClosest`, including distance and `closestHitFraction`. |
| Sleep | `isSleeping` / `wakeUp` / `sleep` and the `allowSleep` force-wake workaround. |
| Volume | Shape sizing and the minimum-volume clamp. |
| Filtering | Group/mask filtering through Rapier's packed interaction groups. |
| Stable 1 | Stacked-box settling at scale 1 — exercises the full `syncSceneToPhysics` → `step` → writeback loop. |
| Stable 0.5 | Same at scale 0.5 — exercises non-unit node scale baked into shape parameters. |
| Basic api | The `IRigidBody` surface: velocities, damping, forces, impulses, torques. |
| Local inertia | Mass distribution across a body's colliders. |
| Use gravity | `useGravity` via Rapier's gravity scale. |
| CCD | `useCCD` / `isUsingCCD`. |

Skipped for `rapier`, because the wrapper slots are deliberately unregistered:
`Sweep`, `Configurable constraint`, `Box character controller`,
`Capsule character controller`.

### Regression checks

```
npx jest tests/physics      → 61 passed, 0 failed   (all 5 backends)
npx jest                    → 160 suites passed, 1 skipped
                              1134 tests passed, 2 skipped, 16 todo
                              34 snapshots passed
npx tsc --noEmit            → 0 errors, repo-wide
npx eslint "cocos/physics/rapier/**/*.ts"  → clean
```

No regressions in any other backend or subsystem. (One warning appears in the full-suite
output — cannon's pre-existing "does not support CapsuleCollider" notice. Unrelated to this
work.)

### Not yet verified

The backend has **not** been run in a live Cocos Creator scene or through an H5 build. The
jest harness drives `PhysicsSystem` directly. Still outstanding:

```
npm run build:dev
npm run build:min     # then check the treemap shows Rapier in its OWN chunk, not cc.js
```

Expected console order at runtime:

```
[rapier]: rapier wasm lib loaded, version 0.20.0.
[PHYSICS]: register rapier.
[PHYSICS]: using rapier.
```

---

## 3. What is implemented

### Backend files — `cocos/physics/rapier/`

| File | Responsibility |
|---|---|
| `instantiated.ts` | Owns the single `RAPIER.init()` promise and the mutable `R` binding. Hooks `game.onPostInfrastructureInitDelegate`. |
| `instantiate.ts` | `selector.register('rapier', {...})` at `Game.EVENT_PRE_SUBSYSTEM_INIT`; exports `loadWasmModuleRapier()`. |
| `rapier-world.ts` | `IPhysicsWorld` — owns `RAPIER.World` and `EventQueue`, stepping, transform sync, event dispatch, raycasts. |
| `rapier-shared-body.ts` | One Rapier body per `Node`, ref-counted; merges one `RigidBody` with N `Collider`s. |
| `rapier-rigid-body.ts` | `IRigidBody` — a facade over the shared body. |
| `rapier-contact-equation.ts` | `IContactEquation` over copied manifold data. |
| `rapier-cache.ts` | Handle → wrapper registries and reusable scratch vectors. |
| `rapier-utils.ts` | Interaction-group packing, axis→rotation, query filter translation, scale helpers. |
| `rapier-enum.ts` | Local mirrors of Rapier's numeric enums, plus the body-type map. |
| `shapes/rapier-shape.ts` | Abstract shape base: lifecycle, centre, sensor flag, material, group/mask, AABB, event opt-in. |
| `shapes/rapier-axial-shape.ts` | Shared base for the Y-axis-only shapes (capsule, cylinder, cone). |
| `shapes/rapier-{box,sphere,capsule,cylinder,cone,trimesh,plane}-shape.ts` | The seven implemented collider types. |

### Registered wrapper slots

✅ `PhysicsWorld`, `RigidBody`, `BoxShape`, `SphereShape`, `CapsuleShape`, `CylinderShape`,
`ConeShape`, `TrimeshShape`, `PlaneShape`

❌ `TerrainShape`, `SimplexShape`, `PointToPointConstraint`, `HingeConstraint`,
`FixedConstraint`, `ConfigurableConstraint`, `BoxCharacterController`,
`CapsuleCharacterController`

### Edits outside the backend

| File | Change |
|---|---|
| `package.json` | `@dimforge/rapier3d-compat` at `0.20.0` (exact, matching the `@cocos/cannon` house style) |
| `cc.config.json` | `features["physics-rapier"]`, `constants.LOAD_RAPIER_MANUALLY` |
| `exports/physics-rapier.ts` | New module entry point |
| `physics-system.ts` | `LOAD_RAPIER_MANUALLY` added to both manual-load guards; new `PHYSICS_RAPIER` getter |
| `physics-selector.ts`, `physics-config.ts` | `'rapier'` added to the id unions (DX only — both already ended in `\| string`) |
| `@types/dispose-symbols.d.ts` | New; see §4.3 |
| `tsconfig.json` | Registers the above |
| `tests/init.ts` | `TextDecoder` / `TextEncoder` polyfill; see §4.3 |
| `tests/constants-for-test.ts` | `LOAD_RAPIER_MANUALLY = false` |
| `tests/physics/physics.test.ts` | Imports the backend, awaits its init, skips unimplemented suites |

Regenerate `@types/consts.d.ts` with `npm run build:const` after touching
`cc.config.json` constants.

### Architecture

Structure mirrors `cocos/physics/cannon/`, the existing precedent for an npm-JS-library
backend. The **SharedBody** pattern is reproduced as every backend requires: one Rapier
body per `Node`, keyed by `node.uuid`, ref-counted.

Two places where this backend is *simpler* than bullet:

- Rapier sensors live on the same body as solid colliders, so there is no ghost object and
  `isTrigger` is just `setSensor`. Bullet has to move the shape between two compounds.
- No `BODY_RE_ADD` dirty flag, because `setCollisionGroups` and `setBodyType` are both live
  in Rapier. The only deferred work is mass recomputation.

Cocos `Vec3`/`Quat` are structurally compatible with Rapier's `Vector`/`Rotation`, and
Rapier's getters accept an in-place `target`, so the transform path is allocation-free —
no marshalling layer like bullet's `cocos2BulletVec3`.

---

## 4. Things to know before touching this code

### 4.1 Never dereference `R` at module scope

Rapier's classes and enums only exist after `init()` resolves, and every module here is
evaluated long before the boot sequence reaches `Game.EVENT_PRE_SUBSYSTEM_INIT`.

```ts
const IDENTITY = new R.Quaternion(0, 0, 0, 1);   // ✗ throws at bundle load
constructor () { this._q = new R.Quaternion(0, 0, 0, 1); }   // ✓
```

This is the single most likely way to break the backend, and the failure presents as an
opaque `Cannot read properties of undefined`. Mitigations in place: the numeric enums are
mirrored locally in `rapier-enum.ts` so the common path never needs `R` early, and
`RapierWorld`'s constructor calls `assertRapierReady()` which reports the problem by name.
An ESLint rule banning `R.` in module-scope initialisers would be worth adding.

### 4.2 Rapier handles are recycled

Collider and body handles are arena indices, and `RAPIER.Collider` has **no `userData`**
field — so the `setWrap`/`getWrap` trick used by cannon and bullet is unavailable. Events
hand back raw integers, resolved through `RapierCache`.

A collider destroyed this frame can hand its handle straight to the next collider created.
**Every removal and rebuild must deregister synchronously**, before anything new is
created, or events will resolve to the wrong component. All removal paths go through
`destroyCollider()` / `RapierSharedBody.destroy()`. Rebuilding a collider (mesh or scale
change) also mints a fresh handle — `rebuildCollider()` handles the del-then-set.

### 4.3 Two toolchain gaps this integration had to patch

- **`Symbol.dispose`.** Rapier's generated wasm-bindgen typings declare `[Symbol.dispose]()`
  on every class. TypeScript ships that symbol from 5.2; this repo is pinned to 4.9, which
  produced ~70 errors from `node_modules`. `@types/dispose-symbols.d.ts` declares the symbol
  as `unique symbol`, which fixes both error flavours. This was chosen over enabling
  `skipLibCheck`, which would have silenced type errors across *every* dependency in the
  engine. Delete the file when the repo moves to TS ≥ 5.2.
- **`TextDecoder` under jest.** `jest-environment-jsdom` does not provide it, and Rapier's
  glue constructs one unconditionally at module scope — so merely *importing* Rapier under
  jest threw before any test ran. Polyfilled from `node:util` at the top of `tests/init.ts`.

### 4.4 Import order in `tests/physics/physics.test.ts`

`physics-cannon` must stay the **last** physics import. `selector.register` makes the
last-registered backend active, and that file constructs the world at *module scope* —
before `beforeAll` has awaited the WASM backends. Cannon is pure JS with no async init, so
it is the only safe default for that module-scope world. Importing rapier after it makes
rapier win, and the world is then built before its WASM has loaded.

### 4.5 Verified Rapier 0.20.0 behaviour

Checked against the installed package rather than assumed. Useful when extending:

- `World.step(eventQueue?, hooks?)` takes **no** delta — `world.timestep` is world state.
- There is **no `ColliderDesc.halfspace`**; planes use `new ColliderDesc(new HalfSpace(n))`.
- `RigidBody` has **no runtime `setCanSleep`**, and no sleep-threshold binding at all.
- `addForce` / `addTorque` **persist** until explicitly reset, unlike Cocos's one-step
  semantics. The world clears them at the tail of each step.
- A zero-mass dynamic body with no colliders is **frozen outright**. `setAdditionalMass`
  restores motion even though `mass()` still reports 0.
- `ActiveCollisionTypes.DEFAULT` (15) excludes fixed↔fixed and kinematic↔fixed, so
  event-subscribed colliders need `ALL` (60943) — otherwise a kinematic character entering
  a static trigger volume is never even narrow-phased.
- `Collider` exposes no AABB, so `getAABB` is computed analytically per shape.
- `TriMeshFlags.FIX_INTERNAL_EDGES = 144` (not a single bit).
- `IntegrationParameters.erp` no longer exists — there is a read-only `contact_erp` and a
  write-only `contact_natural_frequency`. `QueryPipeline` has been removed from `World`.

---

## 5. Known limitations

Places where Rapier's model cannot faithfully honour the Cocos contract. Each is commented
at its site in the code.

| # | Cocos expects | Rapier reality | Mitigation |
|---|---|---|---|
| 1 | 32 collision groups, independent 32-bit group and mask | `InteractionGroups` packs `(membership << 16) \| filter` — **16 bits each** | High bits folded down. Fails permissively (things may collide that shouldn't) rather than letting objects fall through the world. Warns once. |
| 2 | Non-uniform scale on any collider | Colliders have no scale | Exact for box; largest-component approximation for sphere; per-axis for capsule/cylinder/cone; **full O(V) rebuild + BVH re-cook** for trimesh |
| 3 | `setSleepThreshold` | Not bound in the JS API | Stored for round-tripping, warns once. Sleep onset differs from other backends. |
| 4 | `setAllowSleep` at runtime | Only `RigidBodyDesc.setCanSleep`, at creation | Force-wake each step. A body may still sleep for the duration of one step. |
| 5 | `isSleepy` (tri-state) | Boolean only | Always `false` |
| 6 | `linearFactor` / `angularFactor` as scalars | Per-axis on/off locks | `!== 0` → enabled; warns on fractional values |
| 7 | `rollingFriction` / `spinningFriction` | No rolling or spinning friction model | Ignored, warns once. **Spheres will roll forever on flat ground.** |
| 8 | `impl` on events and contacts exposes a native object | `TempContactManifold` is a borrowed view, freed after its callback | All three `impl` fields are `null`; contact data is copied |
| 9 | Sensible inertia for dynamic mesh/plane bodies | Trimesh and half-space have zero mass properties | Point-mass fallback via `setAdditionalMass` |

### Deliberate behavioural choices

- Friction and restitution combine rule is set to **Multiply** to match bullet. Rapier's own
  default is Average; 3.8 projects are tuned against bullet.
- Physics→scene writeback happens at the tail of `step()` (the cannon/physx approach, not
  bullet's motion-state callback), resolved through the handle registry rather than
  `body.userData`.
- `syncAfterEvents` uses a *checked* resync, because the writeback itself dirties
  `node.hasChangedFlags`; an unconditional push would feed the body's own output back into
  the solver and discard its predicted position.
- Registration at `EVENT_PRE_SUBSYSTEM_INIT` is **unconditional**, and must stay that way:
  that event is a one-shot firing before any manual load, so gating it on readiness would
  mean the backend never registers when `LOAD_RAPIER_MANUALLY` is set.

### Inherited wart

The guard in `PhysicsSystem.constructAndRegister` is global, not per-backend. Setting
`LOAD_RAPIER_MANUALLY` therefore also suppresses bullet's and physx's auto-registration.
Pre-existing behaviour, not introduced here; a real fix is a per-backend registry of pending
manual loads.

---

## 6. Not yet implemented

Roughly in the order worth tackling.

### Character controllers + shape sweeps

Best value-to-effort ratio. `world.createCharacterController(offset)` maps almost 1:1 onto
`cocos/physics/spec/i-character-controller.ts`, and brings tuning the Cocos spec cannot
express: autostep, snap-to-ground, slope climb/slide angles,
`applyImpulsesToDynamicBodies`.

Two wrinkles: Rapier's controller is pure kinematic sweeping with no internal rigid body, so
`getPosition`/`setPosition` need a wrapper-side position; and it reports **no** trigger
overlaps, so `onControllerTrigger*` must be synthesised with the same dictionary machinery
the collision events use.

Sweeps come along cheaply — `world.castShape` covers `sweepBoxClosest` /
`sweepSphereClosest` / `sweepCapsuleClosest` directly. The all-hits variants have no Rapier
equivalent (castShape is closest-only) and need a repeated-cast-with-exclusion loop.
Currently all six warn once and return `false`.

### Constraints

`JointData.spherical` / `revolute` / `fixed` map cleanly onto `PointToPointConstraint` /
`HingeConstraint` / `FixedConstraint`, and Rapier's revolute joint supports limits and a
motor.

`ConfigurableConstraint` (6-DOF) is the hardest single piece in the whole port. Cocos models
each of six axes independently as `FREE` / `LIMITED` / `LOCKED` plus a per-axis driver;
Rapier's `JointData.generic` takes only a **locked-axis bitmask**. `LOCKED` and `LIMITED`
are expressible, and the drivers map onto `configureMotorPosition` /
`configureMotorVelocity` — but per-axis **limit softness has no equivalent**, because
Rapier's limits are hard. Expect to ship it with that unsupported.

### Terrain and simplex shapes

Unregistered, so they currently warn and no-op. Rapier has `ColliderDesc.heightfield` for
terrain and `convexHull` for simplex.

### Debug draw

Cheap and high day-to-day value. `world.debugRender()` returns the entire world as one
`{ vertices, colors }` buffer pair — a much better story than bullet's per-line callback.
Feed it to `GeometryRenderer` and respect `EPhysicsDrawFlags`.

### Editor integration

Add a `physics-rapier` entry under the single-select `physics.options` group in
`editor/engine-features/render-config.json`, plus labels in
`editor/i18n/{en,zh}/localization.js`, so the backend is selectable in Cocos Creator's UI
rather than only from code. Do **not** set `isNativeModule` or `cmakeConfig` (web-only), and
do not set `default: true` — `physics-ammo` owns that.

---

## 7. Future work: the deep-Rapier API

The `IPhysics*` spec is a lowest common denominator across four very different engines, so
it cannot express most of what makes Rapier worth adopting. The plan is a
`cocos/physics/rapier/extras/` directory — a side-effect-free barrel re-exported from
`exports/physics-rapier.ts`, so unreferenced parts tree-shake away and the spec
implementation stays clean.

Three tiers:

**1. Escape hatch (already available.)** `impl` on the world, body and shape is part of the
spec and already returns the live `RAPIER.World` / `RigidBody` / `Collider`. Everything
Rapier can do is reachable today, just untyped.

**2. Typed accessors.** `getRapierWorld()`, `getRapierRigidBody(component)`,
`getRapierCollider(component)`, `getRapier()`. Each returns `null` when rapier is not the
active backend, so callers can feature-detect instead of hand-casting `any`.

**3. Named helpers** for what the spec cannot express:

| Area | Surface |
|---|---|
| Solver tuning | `numSolverIterations`, `numInternalPgsIterations`, `maxCcdSubsteps`, `lengthUnit`, `contact_natural_frequency` |
| Per-body | soft CCD, dominance groups, additional solver iterations, additional mass properties |
| Per-collider | `ActiveEvents`, `ActiveCollisionTypes`, `ActiveHooks`, contact skin, density/mass properties, friction and restitution combine rules, solver groups (separate from collision groups) |
| Physics hooks | `filterContactPair` / `filterIntersectionPair` — also the route to true 32-bit collision filtering, at the cost of a JS↔WASM callback per candidate pair per step |
| Events | Contact **force** events (`totalForceMagnitude`, `maxForceDirection`) — no Cocos equivalent at all; good for destructible objects and impact audio |
| Queries | `QueryFilterFlags`, JS predicates, `projectPoint`, `intersectionsWithPoint`, AABB queries, narrow-phase manifold inspection |
| Debug | `debugRender()` buffers |
| Snapshots | `takeSnapshot()` / `restoreSnapshot()` — **mark experimental.** `restoreSnapshot` is a static returning a *new* `World`, so every collider and body handle must be rebound afterwards. Easy to get subtly wrong. |
| Joints | Motors and springs, `JointData.spring` / `rope` / `generic`, multibody (reduced-coordinate) joints for drift-free articulated chains |
| Character controller | The full ~15-knob tuning set |

Do **not** add `@ccclass` components to the main feature — decorator registration is a side
effect that can never tree-shake. If inspector-editable settings are wanted, ship them as a
separate `physics-rapier-components` ccbuild feature depending on `physics-rapier`.

Also deferred: Rapier's `PidController` and `DynamicRayCastVehicleController`. Both are
substantial subsystems and don't fit the "expose what the spec can't" framing.

### Possible variant swaps

Rapier publishes `-simd` and `-deterministic` builds at the same version. Both drop into the
existing loader cleanly, since it already resolves the module through a dynamic `import()`
behind a mutable binding.

- **`-simd`** needs a feature-detect and a second binary, because baseline WASM SIMD is
  missing on older low-end Android WebViews. Not worth it while the payload is already ~1 MB.
- **`-deterministic`** uses soft-float for bit-identical results across machines. Measurably
  slower, and only matters for lockstep netcode — the same use case that wants snapshots.

---

## 8. Using it today

Enable the `physics-rapier` feature in the build, then select the backend:

```ts
import { physics } from 'cc';
physics.selector.switchTo('rapier');
```

Or check which backend is live:

```ts
import { PhysicsSystem } from 'cc';
PhysicsSystem.PHYSICS_RAPIER;                  // true
PhysicsSystem.instance.physicsWorld.impl;      // the live RAPIER.World
```

If the project was built with `LOAD_RAPIER_MANUALLY`, await the module first — typically
behind a loading screen:

```ts
import { loadWasmModuleRapier } from 'cc';
await loadWasmModuleRapier();
```

Expect these three lines, in order:

```
[rapier]: rapier wasm lib loaded, version 0.20.0.
[PHYSICS]: register rapier.
[PHYSICS]: using rapier.
```

Seeing `[PHYSICS]: register rapier.` *before* the wasm line means the boot hook did not
take — check `LOAD_RAPIER_MANUALLY`. Seeing `switch from bullet to rapier` instead of
`using rapier` just means another backend registered first, which is expected when several
physics features are enabled at once.
