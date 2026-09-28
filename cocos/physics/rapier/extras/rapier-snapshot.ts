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
import { warn } from '../../../core';
import { getRapier, getRapierWorld } from './rapier-access';

/**
 * EXPERIMENTAL. Serializes the whole physics world, which is the basis for deterministic
 * replay and rollback netcode - one of the main reasons to pick Rapier over the other
 * backends.
 *
 * Restoring in place is deliberately NOT offered. Rapier's `World.restoreSnapshot` is a
 * static that returns a brand-new `World`, which invalidates every collider and rigid-body
 * handle the Cocos wrappers hold. Rebinding all of them safely is a larger change than an
 * accessor, so this module exposes the bytes and a detached world, and leaves that
 * decision to the caller.
 */
export function rapierTakeSnapshot (): Uint8Array | null {
    const world = getRapierWorld();
    return world ? world.takeSnapshot() : null;
}

/**
 * Builds a NEW detached `RAPIER.World` from `data`.
 *
 * The engine's colliders and bodies keep pointing at the existing world, so this is for
 * inspection, diffing or callers driving Rapier directly - not a way to rewind the scene.
 */
export function rapierWorldFromSnapshot (data: Uint8Array): RAPIER.World | null {
    const R = getRapier();
    if (!R) return null;
    warn('[PHYSICS][rapier]: rapierWorldFromSnapshot returns a detached world; the engine\'s '
        + 'colliders and bodies still reference the previous one.');
    return R.World.restoreSnapshot(data);
}
