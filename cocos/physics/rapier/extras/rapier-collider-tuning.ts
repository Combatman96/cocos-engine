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
import { Collider } from '../../../../exports/physics-framework';
import { getRapierCollider } from './rapier-access';

/* Per-collider capabilities `IBaseShape` cannot express. */

/**
 * A speculative contact margin. A small skin markedly improves stacking stability without
 * visibly separating bodies.
 */
export function setRapierContactSkin (collider: Collider, thickness: number): void {
    getRapierCollider(collider)?.setContactSkin(thickness);
}

export function getRapierContactSkin (collider: Collider): number {
    return getRapierCollider(collider)?.contactSkin() ?? 0;
}

/**
 * Solver groups are separate from collision groups: a pair filtered out here still
 * generates contact events but produces no forces. The Cocos group/mask pair maps onto
 * collision groups only, so this is the only way to reach the distinction.
 */
export function setRapierSolverGroups (collider: Collider, groups: number): void {
    getRapierCollider(collider)?.setSolverGroups(groups);
}

/**
 * Which event kinds this collider produces, as a combination of `ERapierActiveEvents`.
 *
 * Note the backend drives this itself from the collider's Cocos listeners, so a value set
 * here is overwritten the next time a listener is added or removed. Re-apply it after any
 * such change if contact-force events must stay on.
 */
export function setRapierActiveEvents (collider: Collider, events: number): void {
    getRapierCollider(collider)?.setActiveEvents(events as RAPIER.ActiveEvents);
}

/**
 * Opts this collider into the physics hooks. A hook installed without this flag silently
 * never fires, which is the single most common Rapier support question.
 */
export function setRapierActiveHooks (collider: Collider, hooks: number): void {
    getRapierCollider(collider)?.setActiveHooks(hooks as RAPIER.ActiveHooks);
}

/**
 * Which body-type pairs are narrow-phased for this collider. Rapier's DEFAULT excludes
 * fixed/fixed and kinematic/fixed; ALL is needed for, say, a kinematic character entering
 * a static trigger volume.
 */
export function setRapierActiveCollisionTypes (collider: Collider, types: number): void {
    getRapierCollider(collider)?.setActiveCollisionTypes(types as RAPIER.ActiveCollisionTypes);
}

/** Minimum total force before a contact-force event fires for this collider. */
export function setRapierContactForceEventThreshold (collider: Collider, threshold: number): void {
    getRapierCollider(collider)?.setContactForceEventThreshold(threshold);
}

/**
 * Rapier combines the two colliders' coefficients with a rule. The backend defaults to
 * Multiply to match bullet, whereas Rapier's own default is Average; `PhysicsMaterial` has
 * no notion of a combine rule at all.
 */
export function setRapierFrictionCombineRule (collider: Collider, rule: number): void {
    getRapierCollider(collider)?.setFrictionCombineRule(rule as RAPIER.CoefficientCombineRule);
}

export function setRapierRestitutionCombineRule (collider: Collider, rule: number): void {
    getRapierCollider(collider)?.setRestitutionCombineRule(rule as RAPIER.CoefficientCombineRule);
}
