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
import { Collider, PhysicsSystem } from '../../../../exports/physics-framework';
import { RapierCache } from '../rapier-cache';
import type { RapierWorld } from '../rapier-world';
import { ERapierSolverFlags } from './rapier-enums';
import { isRapierActive } from './rapier-access';

/**
 * Per-step contact and intersection filtering, which the Cocos group/mask pair cannot
 * express. This is also the route to true 32-bit collision filtering, since Rapier's
 * interaction groups are only 16 bits wide per half.
 *
 * Rapier hands the callbacks raw integer handles; this adapter resolves them to Cocos
 * components. Both members are optional here even though Rapier's own interface requires
 * both - the adapter supplies a permissive default for whichever is missing.
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
 * Two costs worth knowing. These callbacks cross the JS/WASM boundary for every candidate
 * pair, every step, so restrict them to the colliders that need it through
 * `setRapierActiveHooks`. And a hook installed without that flag silently never fires.
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
