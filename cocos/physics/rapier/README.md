# Rapier physics backend

A fifth 3D physics backend for this fork of cocos-engine 3.8.8, backed by
[Rapier](https://rapier.rs) — a Rust physics engine compiled to WebAssembly with official
JS bindings. It sits alongside the four backends the engine already ships: `bullet`,
`physx`, `cannon.js` and `builtin`.

- **Status:** complete. Every `IPhysicsWrapperObject` slot is registered and all ten
  `tests/physics` suites run against it.
- **Selector id:** `rapier`
- **Dependency:** `@cocos/rapier3d-compat`, an npm alias of
  `@dimforge/rapier3d-compat@0.20.0`
- **Platforms:** web / H5 only
- **Size:** 41 TypeScript files, ~6,000 lines

---

## 1. Why this exists

Rapier is actively developed and has a modern solver, but the motivation is a set of
capabilities none of the existing backends expose: deterministic stepping, world snapshots
(the basis for rollback netcode), a tunable solver, user-supplied physics hooks, and a real
kinematic character controller. Those live in [`extras/`](extras/) — see §7.

Scope decisions:

- **Web / H5 only.** No JSB, no CMake, no mini-game subpackaging. Rapier is Rust, so a
  native Cocos binding would be a separate and far larger effort.
- **Additive.** Registers under a new id; nothing about the other four backends changes.

Adding a backend is cheap because `cocos/physics/framework/physics-selector.ts` is the
entire integration point — a backend is one `selector.register('rapier', {...})` call.

### Packaging, and the three resolution problems it had to solve

The package is an npm dependency, installed under the alias `@cocos/rapier3d-compat`.
`scripts/patch-rapier-package.js` runs from `postinstall` and does two things, each
fixing a failure that only appeared inside Cocos Creator.

**1. Creator could not compile the engine.**

```
Could not resolve './exports' from node_modules/@cocos/rapier3d-compat/dist/rapier.d.ts
```

Creator resolves the engine's external dependencies to a file and hands that file to
Rollup. Rapier ships a `types` field, so Creator selected `dist/rapier.d.ts` as the module
to bundle, and Rollup died on its `import * as RAPIER from ./exports` — `exports.d.ts`
has no `.js` twin. `@cocos/cannon` and `@cocos/box2d` declare no `types` at all, which is
why they have always worked. The script deletes `types`/`typings` so the specifier resolves
to real JavaScript; `@types/rapier3d-compat.d.ts` restores the typings through an ambient
declaration, since module resolution and type lookup are independent in TypeScript.

**2. The editor could not resolve the module at runtime.**

```
Unable to resolve bare specifier '@cocos/rapier3d-compat'   (SystemJS Error#8)
```

Creator emits external npm dependencies into `editor/external/` — `%40cocos/cannon.js` and
so on — but never emitted one for Rapier, whether the import was static or dynamic. The
script therefore copies `dist/rapier.mjs` to `native/external/rapier/rapier.js`, and the
loader imports it as `external:rapier/rapier.js`. ccbuild's `externalWasmLoader` turns any
`external:` id that matches no suffix rule into a real SystemJS module — the fallback at
the end of its `_load` — which is exactly how the bullet backend loads its wasm, and the
only route that works with a dynamic import inside Creator. `native/external/` is
gitignored, so the copy is regenerated on every install.

**3. Physics did not run in Preview in Editor.** Not a packaging problem, but the same
class of mistake: the loader had been gated on `EDITOR_NOT_IN_PREVIEW`, which suppressed
it inside the editor. That fails silently — no world, no bodies, no log, no error. The
registration is now unconditional, matching bullet.

Measured from `npm run build:dev`:

| Chunk | raw | gzip |
|---|---:|---:|
| `rapier-*.js` (wasm module, own chunk) | 2.91 MB | ~1.04 MB |
| `physics-rapier.js` (backend code) | 170 KB | ~25 KB |

The wasm stays in its own lazily-imported chunk, so a project that ships the feature but
never constructs a rapier world never downloads it, and `LOAD_RAPIER_MANUALLY` is
meaningful.

Rejected alternative: vendoring into `native/external/` by hand. That tree is pinned by
`native/external-config.json` and replaced wholesale by `npm run update:native-external`,
so the copy has to be generated at install time rather than committed.

## 2. Test results

```
npx jest tests/physics    # all ten suites, five backends
npx jest                  # 162 suites, 1167 tests, 0 failures
npx tsc --noEmit          # 0 errors repo-wide
npx eslint "cocos/physics/rapier/**/*.ts"
npm run build:dev         # H5 build, chunk split verified
```

`tests/physics/physics.test.ts` runs every suite against `rapier` with no backend guards.
Two rapier-specific files cover what the shared suites cannot:

- `tests/physics/rapier-internals.test.ts` — teardown ordering, handle deregistration,
  sweep edge cases, each constraint's behaviour, character-controller thresholds, the
  terrain transposition, debug draw, and the editor feature entry.
- `tests/physics/rapier-extras.test.ts` — the `extras/` surface, including asserting every
  local enum mirror against the real Rapier value.

**Not yet verified:** the backend has not been run in a live Cocos Creator scene. Jest
drives `PhysicsSystem` directly and jsdom has no camera, so debug draw is covered only as
far as its buffers and its no-camera path.

Expected console order at runtime:

```
[rapier]: rapier wasm lib loaded, version 0.20.0.
[PHYSICS]: register rapier.
[PHYSICS]: using rapier.
```

---

## 3. What is implemented

Every wrapper slot: `PhysicsWorld`, `RigidBody`, all nine collider shapes, all four
constraints, and both character controllers.

| Area | Files |
|---|---|
| Loader / registration | `instantiated.ts`, `instantiate.ts` |
| World | `rapier-world.ts` — stepping, sync, events, raycast, sweeps, debug draw |
| Bodies | `rapier-shared-body.ts`, `rapier-rigid-body.ts`, `rapier-contact-equation.ts` |
| Infrastructure | `rapier-cache.ts`, `rapier-utils.ts`, `rapier-enum.ts` |
| Shapes | `shapes/` — base, axial base, box, sphere, capsule, cylinder, cone, trimesh, plane, terrain, simplex |
| Constraints | `constraints/` — base, point-to-point, hinge, fixed, configurable |
| Character controllers | `character-controllers/` — base, box, capsule |
| Rapier-only API | `extras/` — 11 modules, see §7 |

---

## 4. Things to know before touching this code

### 4.1 Never dereference `R` at module scope

Rapier's classes and enums only exist after `init()` resolves, and every module here is
evaluated long before that.

```ts
const IDENTITY = new R.Quaternion(0, 0, 0, 1);   // ✗ throws at bundle load
constructor () { this._q = new R.Quaternion(0, 0, 0, 1); }   // ✓
```

Mitigations: the numeric enums are mirrored in `rapier-enum.ts` so the common path never
needs `R` early, and `RapierWorld`'s constructor calls `assertRapierReady()` which reports
the problem by name instead of an opaque property access.

### 4.2 Rapier handles are recycled

Collider and body handles are arena indices, and `RAPIER.Collider` has **no `userData`** —
so the `setWrap`/`getWrap` trick the cannon and bullet backends use is unavailable. Events
hand back raw integers, resolved through `RapierCache`.

A collider destroyed this frame can hand its handle straight to the next one created, so
**every removal and rebuild deregisters synchronously**. Rebuilding a collider (mesh, scale,
terrain or simplex change) also mints a fresh handle.

### 4.3 Teardown order is load-bearing

`selector.switchTo()` destroys the world while the scene's components are still alive, so
everything that holds a Rapier handle has a re-entry guard and checks `RapierWorld.destroyed`
before calling in. `RapierWorld.destroy()` tears down in dependency order: character
controllers, then joints, then bodies — removing a body first would leave its joints holding
freed parents.

This is not hypothetical. Both the shared body and the character controller shipped bugs of
exactly this shape, and the second one cascaded into 11 failures in the **cannon.js** suite
that runs after rapier.

### 4.4 Two toolchain gaps this integration patched

- **`Symbol.dispose`.** Rapier's generated typings declare `[Symbol.dispose]()` on every
  class. TypeScript ships that symbol from 5.2; this repo is pinned to 4.9, producing ~70
  errors from `node_modules`. `@types/dispose-symbols.d.ts` declares it as `unique symbol`.
  Chosen over `skipLibCheck`, which would silence type errors across every dependency.
  Delete the file when the repo moves to TS ≥ 5.2.
- **`TextDecoder` under jest.** `jest-environment-jsdom` does not provide it, and Rapier's
  glue constructs one at module scope — so merely *importing* Rapier under jest threw.
  Polyfilled from `node:util` in `tests/init.ts`.

- **Package `exports` map lists `types` first.** Rapier's `package.json` declares
  `{ "types": "./dist/rapier.d.ts", "require": "...cjs", "import": "...mjs" }`. A
  bundler that honours the `types` condition — Cocos Creator's engine compiler does — resolves
  the bare specifier to `rapier.d.ts` and then fails on its `export * from "./exports"`,
  which has no `.js` twin. The error reads `Could not resolve './exports' from
  .../dist/rapier.d.ts`. Importing through the `@cocos/rapier3d-compat` npm alias sends the
  package through Creator's node-module bundler, whose `require.resolve` selects the runtime
  CommonJS entry and emits a browser-addressable external module. The postinstall script
  `scripts/patch-rapier-package.js` also moves the runtime export conditions before `types`;
  Creator enables both conditions and follows insertion order. `jest.config.js` maps the
  alias to `dist/rapier.cjs`.

- **Creator preview cannot dynamically import engine filesystem modules.** They resolve to
  `q-bundled://` URLs, but preview is served from `http://localhost`, so Chromium blocks the
  request under CORS. The `@cocos/*` npm alias makes Creator emit Rapier under `external/`
  instead, preserving lazy loading without crossing protocols.

### 4.5 Import order in `tests/physics/physics.test.ts`

`physics-cannon` must stay the **last** physics import. `selector.register` makes the
last-registered backend active, and that file constructs the world at *module scope*, before
`beforeAll` has awaited the WASM backends. Cannon is pure JS with no async init, so it is the
only safe default for that world.

### 4.6 Rapier 0.20.0 behaviour that contradicts its own typings

Verified against the installed package, not assumed:

- **`JointData.limitsEnabled` / `.limits` are silently ignored for revolute joints.** A joint
  built that way reports `limitsEnabled() === false` and ±FLT_MAX bounds. Limits must go
  through `RevoluteImpulseJoint.setLimits()` after creation. There is no call to switch them
  back off, so disabling rebuilds the joint.
- `World.step(eventQueue?, hooks?)` takes **no** delta; `world.timestep` is world state.
- There is **no `ColliderDesc.halfspace`**; planes use `new ColliderDesc(new HalfSpace(n))`.
- `RigidBody` has **no runtime `setCanSleep`** and no sleep-threshold binding.
- `addForce`/`addTorque` **persist** until reset, unlike Cocos's one-step semantics.
- A zero-mass dynamic body with no colliders is **frozen outright**; `setAdditionalMass`
  restores motion even though `mass()` still reports 0.
- `ActiveCollisionTypes.DEFAULT` (15) excludes fixed↔fixed and kinematic↔fixed, so
  event-subscribed colliders need `ALL` (60943).
- `Collider` exposes no AABB, so `getAABB` is computed analytically per shape.
- Heightfield buffers are **column-major** and `nrows`/`ncols` are **cell** counts.
- `IntegrationParameters.erp` no longer exists — `contact_erp` is read-only and
  `contact_natural_frequency` write-only. `QueryPipeline` is gone from `World`.

---

## 5. Known limitations

| # | Cocos expects | Rapier reality | Mitigation |
|---|---|---|---|
| 1 | 32 collision groups, independent 32-bit group and mask | `InteractionGroups` packs `(membership << 16) \| filter` — **16 bits each** | High bits folded down. Fails permissively (things may collide that shouldn't) rather than letting objects fall through the world. Warns once. `setRapierPhysicsHooks` gives exact filtering at a per-pair cost. |
| 2 | Non-uniform scale on any collider | Colliders have no scale | Exact for box; largest-component approximation for sphere; per-axis for capsule/cylinder/cone; **full O(V) rebuild** for trimesh, terrain and simplex |
| 3 | `setSleepThreshold` | Not bound in the JS API | Stored for round-tripping, warns once |
| 4 | `setAllowSleep` at runtime | Only `RigidBodyDesc.setCanSleep`, at creation | Force-wake each step; a body may still sleep for one step |
| 5 | `isSleepy` (tri-state) | Boolean only | Always `false` |
| 6 | `linearFactor`/`angularFactor` as scalars | Per-axis on/off locks | `!== 0` → enabled; warns on fractional values |
| 7 | `rollingFriction`/`spinningFriction` | No such model | Ignored, warns once. **Spheres roll forever on flat ground.** |
| 8 | `impl` on events and contacts | `TempContactManifold` is a borrowed view freed after its callback | All three `impl` fields are `null`; contact data is copied |
| 9 | Dynamic mesh/plane inertia | Trimesh and half-space have zero mass properties | Point-mass fallback via `setAdditionalMass` |
| 10 | `ConfigurableConstraint` 6-DOF limits and drivers | `JointData.generic` takes only a locked-axis mask; `GenericImpulseJoint` has **no members** | LOCKED and FREE exact; **LIMITED degrades to FREE**, all drivers inert, soft-constraint stiffness/damping/restitution ignored. Warns once each. |
| 11 | `FixedConstraint.breakForce` / `breakTorque` | Rapier joints never break | Warns once, no-op |
| 12 | `onControllerTrigger*` | The character controller reports contacts only | Unsupported; `onControllerColliderHit` works |

### Deliberate choices

- Friction and restitution combine rule is **Multiply**, matching bullet. Rapier's own
  default is Average; 3.8 projects are tuned against bullet.
- Physics→scene writeback happens at the tail of `step()` (the cannon/physx approach),
  resolved through the handle registry rather than `body.userData`.
- `syncAfterEvents` uses a *checked* resync, because the writeback itself dirties
  `node.hasChangedFlags`; an unconditional push would feed the body's own output back into
  the solver.
- Registration at `EVENT_PRE_SUBSYSTEM_INIT` is **unconditional**. That event is a one-shot
  firing before any manual load, so gating it on readiness would mean the backend never
  registers when `LOAD_RAPIER_MANUALLY` is set.

### Inherited wart

The guard in `PhysicsSystem.constructAndRegister` is global, not per-backend. Setting
`LOAD_RAPIER_MANUALLY` also suppresses bullet's and physx's auto-registration. Pre-existing
behaviour, not introduced here.

---

## 6. Using it

Enable the `physics-rapier` feature in the build, then select the backend:

```ts
import { physics, PhysicsSystem } from 'cc';

physics.selector.switchTo('rapier');
PhysicsSystem.PHYSICS_RAPIER;                  // true
PhysicsSystem.instance.physicsWorld.impl;      // the live RAPIER.World
```

With `LOAD_RAPIER_MANUALLY`, await the module first — typically behind a loading screen:

```ts
import { loadWasmModuleRapier } from 'cc';
await loadWasmModuleRapier();
```

---

## 7. The Rapier-only API — `extras/`

A side-effect-free barrel re-exported from `exports/physics-rapier.ts`, so unreferenced
parts tree-shake away. Everything here is capability the `IPhysics*` spec — a lowest common
denominator across four very different engines — cannot express.

| Module | What it adds |
|---|---|
| `rapier-access` | Typed `getRapierWorld/RigidBody/Collider/CharacterController`, `getRapier()`, `isRapierActive()`. Each returns `null` off-backend so callers feature-detect instead of casting `impl`. |
| `rapier-enums` | Public names for the local enum mirrors. Asserted against the real values by the test suite. |
| `rapier-solver-config` | `numSolverIterations`, `numInternalPgsIterations`, `maxCcdSubsteps`, `lengthUnit`, normalized error tolerances, `contactNaturalFrequency`. |
| `rapier-body-tuning` | Soft CCD, dominance groups, per-body solver iterations, additional mass properties. |
| `rapier-collider-tuning` | Contact skin, solver groups (distinct from collision groups), active hooks/events/collision types, contact-force threshold, combine rules. |
| `rapier-hooks` | `filterContactPair` / `filterIntersectionPair`. Also the route to true 32-bit collision filtering. |
| `rapier-events` | Contact-**force** events, which `ICollisionEvent` has no equivalent for. |
| `rapier-queries` | `projectPoint`, AABB enumeration, manifold inspection, and a filter carrying flags, exclusions and an arbitrary predicate. |
| `rapier-debug-render` | The raw line-list buffers, for drawing debug geometry yourself. |
| `rapier-snapshot` | `takeSnapshot()` — the basis for rollback netcode. **Experimental.** |
| `rapier-joints` | Spring, rope and generic impulse joints, plus a reduced-coordinate multibody joint. |
| `rapier-character` | The ~12 controller knobs the component cannot reach: autostep, snap-to-ground, slide, slope angles, impulse transfer, character mass. |

Two caveats:

- **Snapshot restore is not offered in place.** `World.restoreSnapshot` is a static returning
  a *new* `World`, which invalidates every handle the wrappers hold. `rapierWorldFromSnapshot`
  returns that detached world and leaves rebinding to the caller.
- **`setRapierActiveEvents` is overwritten** whenever a Cocos listener is added or removed,
  because the backend drives `ActiveEvents` from the collider's own subscriptions. Re-apply it
  after any such change.

Solver settings write live `World` state, so they must be re-applied after
`selector.switchTo()` rebuilds the world.

---

## 8. Possible follow-ups

- **`-simd` / `-deterministic` builds.** Rapier publishes both at the same version.
  `-simd` needs a feature-detect and a second binary; `-deterministic` uses
  soft-float for bit-identical cross-machine results and is what rollback netcode actually
  wants alongside snapshots.
- **Exact 32-bit collision filtering by default**, once the per-pair hook cost is measured.
- **`PidController` and `DynamicRayCastVehicleController`**, both substantial subsystems.
- **Inspector-editable settings** as a separate `physics-rapier-components` ccbuild feature.
  They must not go in the main feature: `@ccclass` registration is a side effect that can
  never tree-shake.
