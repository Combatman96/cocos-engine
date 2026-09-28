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

import { ERigidBodyType } from '../framework/physics-enum';

/**
 * Mirrors of the Rapier numeric enums we need at module-evaluation time.
 *
 * Rapier's own enum objects only exist once the wasm module has been instantiated, and
 * every module here is evaluated long before that. Re-declaring the values locally keeps
 * module scope safe and, just as importantly, avoids a static `import` of
 * '@dimforge/rapier3d-compat' which would drag the whole wasm payload out of its own
 * lazily-loaded chunk and into the main engine chunk.
 *
 * `tests/physics/rapier-enums.test.ts` asserts these against the real values after init,
 * so a Rapier upgrade that renumbers anything fails loudly rather than silently.
 */

/** Mirror of `RAPIER.RigidBodyType`. */
export const ERapierBodyType = {
    DYNAMIC: 0,
    FIXED: 1,
    KINEMATIC_POSITION_BASED: 2,
    KINEMATIC_VELOCITY_BASED: 3,
} as const;

/** Mirror of `RAPIER.ActiveEvents`. */
export const ERapierActiveEvents = {
    NONE: 0,
    COLLISION_EVENTS: 1,
    CONTACT_FORCE_EVENTS: 2,
} as const;

/**
 * Mirror of `RAPIER.ActiveCollisionTypes`.
 *
 * Note `DEFAULT` (15) covers only DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED.
 * It deliberately excludes FIXED_FIXED, KINEMATIC_FIXED and KINEMATIC_KINEMATIC, which
 * means a kinematic character walking into a static trigger volume would never even be
 * narrow-phased. Colliders that have event listeners therefore need `ALL`.
 */
export const ERapierActiveCollisionTypes = {
    DEFAULT: 15,
    ALL: 60943,
} as const;

/** Mirror of `RAPIER.QueryFilterFlags`. */
export const ERapierQueryFilterFlags = {
    EXCLUDE_FIXED: 1,
    EXCLUDE_KINEMATIC: 2,
    EXCLUDE_DYNAMIC: 4,
    EXCLUDE_SENSORS: 8,
    EXCLUDE_SOLIDS: 16,
    ONLY_DYNAMIC: 3,
    ONLY_KINEMATIC: 5,
    ONLY_FIXED: 6,
} as const;

/**
 * Mirror of `RAPIER.TriMeshFlags`.
 *
 * `FIX_INTERNAL_EDGES` is not a single bit (it implies ORIENTED | MERGE_DUPLICATE_VERTICES);
 * it makes contact normals account for adjacent triangles, which removes most of the
 * "ghost collision" snagging that trimesh floors otherwise cause.
 */
export const ERapierTriMeshFlags = {
    DELETE_BAD_TOPOLOGY_TRIANGLES: 4,
    ORIENTED: 8,
    MERGE_DUPLICATE_VERTICES: 16,
    DELETE_DEGENERATE_TRIANGLES: 32,
    DELETE_DUPLICATE_TRIANGLES: 64,
    FIX_INTERNAL_EDGES: 144,
} as const;

/** Mirror of `RAPIER.CoefficientCombineRule`. */
export const ERapierCoefficientCombineRule = {
    AVERAGE: 0,
    MIN: 1,
    MULTIPLY: 2,
    MAX: 3,
} as const;

/**
 * Deferred work flushed once per `syncSceneToPhysics`, so that N shape mutations in a
 * single frame cost one mass recompute rather than N.
 *
 * Unlike the bullet backend (`EBtSharedBodyDirty`) there is no BODY_RE_ADD flag: Rapier's
 * `setCollisionGroups` and `setBodyType` are both live, so nothing here requires removing
 * and reinserting the body.
 */
export enum ERapierSharedBodyDirty {
    MASS_PROPERTIES = 1,
}

/**
 * Maps the Cocos body type onto Rapier's.
 *
 * `KinematicPositionBased` is the correct target rather than `KinematicVelocityBased`
 * because Cocos drives kinematic bodies by writing `node.worldPosition` every frame.
 */
export function toRapierBodyType (v: ERigidBodyType): number {
    switch (v) {
    case ERigidBodyType.DYNAMIC:
        return ERapierBodyType.DYNAMIC;
    case ERigidBodyType.KINEMATIC:
        return ERapierBodyType.KINEMATIC_POSITION_BASED;
    case ERigidBodyType.STATIC:
    default:
        return ERapierBodyType.FIXED;
    }
}

/**
 * Upper bound on the repeated-cast loop used for all-hits shape sweeps. Rapier's
 * `castShape` is closest-only, so each iteration has to exclude the colliders already
 * found; without a cap a filter that never converges would spin forever.
 */
export const RAPIER_MAX_SWEEP_HITS = 64;

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
 * The axis-index convention shared by `setConstraintMode` and `setDriverMode`:
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
