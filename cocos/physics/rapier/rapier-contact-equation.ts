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

import { IVec3Like, Quat, Vec3 } from '../../core';
import { ICollisionEvent, IContactEquation } from '../framework/physics-interface';

/**
 * A single contact point of a collision event.
 *
 * Unlike the cannon and bullet equivalents, this holds **copied** values rather than a
 * reference to a native object. Rapier hands out contact data through
 * `TempContactManifold`, a single reused view whose backing pointer is only valid for the
 * duration of the `contactPair` callback, so retaining it would read freed wasm memory.
 *
 * Consequence: `impl` is always null for this backend. Scripts that reach into
 * `event.contacts[i].impl` under bullet or cannon will get null here.
 */
export class RapierContactEquation implements IContactEquation {
    get impl (): null {
        return null;
    }

    isBodyA = false;
    event: ICollisionEvent;

    /**
     * Whether this manifold's collider1/collider2 are swapped relative to the (a, b) pair
     * the world gathered it for. Kept so `isBodyA` can be recomputed per recipient without
     * losing the manifold's own orientation.
     */
    flipped = false;

    /** Contact point in each collider's local frame. */
    readonly localPointA = new Vec3();
    readonly localPointB = new Vec3();
    /** Contact normal in each collider's local frame. */
    readonly localNormalA = new Vec3();
    readonly localNormalB = new Vec3();
    /** Contact normal in world space, pointing from collider 1 towards collider 2. */
    readonly worldNormal = new Vec3();

    /** Collider world transforms captured at the time the contact was read. */
    readonly worldPosA = new Vec3();
    readonly worldPosB = new Vec3();
    readonly worldRotA = new Quat();
    readonly worldRotB = new Quat();

    /** Signed distance between the two surfaces; negative means penetrating. */
    distance = 0;
    /** Normal impulse the solver applied at this point. */
    impulse = 0;

    constructor (event: ICollisionEvent) {
        this.event = event;
    }

    getLocalPointOnA (out: IVec3Like): void {
        Vec3.copy(out, this.isBodyA ? this.localPointA : this.localPointB);
    }

    getLocalPointOnB (out: IVec3Like): void {
        Vec3.copy(out, this.isBodyA ? this.localPointB : this.localPointA);
    }

    getWorldPointOnA (out: IVec3Like): void {
        this._toWorld(out, this.isBodyA);
    }

    getWorldPointOnB (out: IVec3Like): void {
        this._toWorld(out, !this.isBodyA);
    }

    getLocalNormalOnA (out: IVec3Like): void {
        Vec3.copy(out, this.isBodyA ? this.localNormalA : this.localNormalB);
    }

    getLocalNormalOnB (out: IVec3Like): void {
        Vec3.copy(out, this.isBodyA ? this.localNormalB : this.localNormalA);
    }

    getWorldNormalOnA (out: IVec3Like): void {
        Vec3.copy(out, this.worldNormal);
        if (this.isBodyA) Vec3.negate(out, out);
    }

    getWorldNormalOnB (out: IVec3Like): void {
        Vec3.copy(out, this.worldNormal);
        if (!this.isBodyA) Vec3.negate(out, out);
    }

    /** Transforms a stored collider-local contact point into world space. */
    private _toWorld (out: IVec3Like, useFirst: boolean): void {
        const local = useFirst ? this.localPointA : this.localPointB;
        const rot = useFirst ? this.worldRotA : this.worldRotB;
        const pos = useFirst ? this.worldPosA : this.worldPosB;
        Vec3.transformQuat(out as Vec3, local, rot);
        Vec3.add(out as Vec3, out as Vec3, pos);
    }
}
