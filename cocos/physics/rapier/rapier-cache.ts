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

import { Quat, Vec3 } from '../../core';
import type { RapierShape } from './shapes/rapier-shape';
import type { RapierSharedBody } from './rapier-shared-body';

/**
 * Handle -> wrapper registries.
 *
 * Rapier identifies colliders and rigid bodies by integer handles, and reports those raw
 * handles (not objects) through `EventQueue.drainCollisionEvents` and the physics hooks.
 * `RAPIER.Collider` additionally has no `userData` field, so the `setWrap`/`getWrap`
 * approach used by the cannon and bullet backends is unavailable here.
 *
 * WARNING: Rapier handles are recycled arena indices. A collider destroyed this frame can
 * hand its handle straight to the next collider created. Every removal path must therefore
 * call `delShape`/`delBody` synchronously, before any new collider or body is created,
 * otherwise events will resolve to the wrong component. Rebuilding a collider (mesh or
 * scale change) also mints a new handle, so rebuilds must del-then-set.
 */
export class RapierCache {
    private static readonly _shapes = new Map<number, RapierShape>();
    private static readonly _bodies = new Map<number, RapierSharedBody>();

    static setShape (handle: number, shape: RapierShape): void {
        RapierCache._shapes.set(handle, shape);
    }

    static getShape (handle: number): RapierShape | undefined {
        return RapierCache._shapes.get(handle);
    }

    static delShape (handle: number): void {
        RapierCache._shapes.delete(handle);
    }

    static setBody (handle: number, body: RapierSharedBody): void {
        RapierCache._bodies.set(handle, body);
    }

    static getBody (handle: number): RapierSharedBody | undefined {
        return RapierCache._bodies.get(handle);
    }

    static delBody (handle: number): void {
        RapierCache._bodies.delete(handle);
    }
}

/**
 * Reusable scratch values.
 *
 * Cocos `Vec3`/`Quat` are structurally compatible with Rapier's `Vector`/`Rotation`
 * interfaces (`{x,y,z}` and `{x,y,z,w}`), so they can be handed to Rapier directly and
 * passed as the optional `target` parameter of Rapier getters. That makes the whole
 * transform path allocation-free, unlike the bullet backend which has to marshal through
 * wasm heap pointers.
 */
export const CC_V3_0 = new Vec3();
export const CC_V3_1 = new Vec3();
export const CC_V3_2 = new Vec3();
export const CC_QUAT_0 = new Quat();
export const CC_QUAT_1 = new Quat();
