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

/*
 * Public names for the Rapier enum mirrors.
 *
 * These are local copies rather than a static re-export of the Rapier package, which would
 * drag the whole wasm chunk into whatever chunk references them and would also be unusable
 * at module-evaluation time. `tests/physics/rapier-extras.test.ts` asserts every value
 * against the real enum once init() has resolved, so a renumbering upstream fails loudly.
 */

export {
    ERapierActiveEvents,
    ERapierActiveCollisionTypes,
    ERapierQueryFilterFlags,
    ERapierCoefficientCombineRule,
    ERapierJointAxesMask,
    ERapierTriMeshFlags,
    ERapierHeightFieldFlags,
} from '../rapier-enum';

/** Mirror of `RAPIER.ActiveHooks`. Note the member names are plural. */
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
