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

import { IVec3Like } from '../../../core';
import { RigidBody } from '../../../../exports/physics-framework';
import { getRapierRigidBody } from './rapier-access';

/*
 * Per-body capabilities `IRigidBody` cannot express. Free functions rather than a wrapper
 * class, so each one tree-shakes on its own and none of them allocate.
 */

/** Continuous collision detection for this body. `IRigidBody.useCCD` already covers this. */
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

export function getRapierSoftCcdPrediction (body: RigidBody): number {
    return getRapierRigidBody(body)?.softCcdPrediction() ?? 0;
}

/**
 * A body in a higher dominance group is treated as infinite-mass by lower ones, so it
 * pushes them but is never pushed back — a player who shoves crates without being shoved.
 * Range is [-127, 127].
 */
export function setRapierDominanceGroup (body: RigidBody, group: number): void {
    getRapierRigidBody(body)?.setDominanceGroup(group);
}

export function getRapierDominanceGroup (body: RigidBody): number {
    return getRapierRigidBody(body)?.dominanceGroup() ?? 0;
}

/** Extra solver iterations for this body alone, e.g. one ragdoll without a global cost. */
export function setRapierAdditionalSolverIterations (body: RigidBody, iterations: number): void {
    getRapierRigidBody(body)?.setAdditionalSolverIterations(iterations);
}

/**
 * Mass properties beyond the single scalar `RigidBody.mass`: an explicit centre of mass
 * and principal angular inertia, added on top of whatever the colliders contribute.
 */
export function setRapierAdditionalMassProperties (
    body: RigidBody,
    mass: number,
    centerOfMass: IVec3Like,
    principalAngularInertia: IVec3Like,
    angularInertiaLocalFrame: { x: number; y: number; z: number; w: number },
    wakeUp = true,
): void {
    getRapierRigidBody(body)?.setAdditionalMassProperties(
        mass,
        centerOfMass,
        principalAngularInertia,
        angularInertiaLocalFrame,
        wakeUp,
    );
}
