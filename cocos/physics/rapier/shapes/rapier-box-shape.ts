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

import { Vec3 } from '../../../core';
import { BoxCollider } from '../../../../exports/physics-framework';
import { IBoxShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';
import { ERapierSharedBodyDirty } from '../rapier-enum';

const halfExtents = new Vec3();

/** @mangle */
export class RapierBoxShape extends RapierShape implements IBoxShape {
    get collider (): BoxCollider {
        return this._collider as BoxCollider;
    }

    protected onComponentSet (): void {
        this._computeHalfExtents(halfExtents);
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._desc = this.rapier.ColliderDesc.cuboid(halfExtents.x, halfExtents.y, halfExtents.z);
    }

    updateSize (): void {
        this._computeHalfExtents(halfExtents);
        if (this._impl) this._impl.setHalfExtents(halfExtents);
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    protected updateScale (): void {
        this.updateSize();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        return this._computeHalfExtents(out);
    }

    protected getLocalBoundingRadius (): number {
        return this._computeHalfExtents(halfExtents).length();
    }

    /**
     * A box is the one shape where a non-uniform node scale maps exactly onto Rapier,
     * because each half-extent takes its own scale component.
     */
    private _computeHalfExtents (out: Vec3): Vec3 {
        const size = this.collider.size;
        const ws = this._collider.node.worldScale;
        out.x = this.clampSize(size.x * 0.5 * ws.x);
        out.y = this.clampSize(size.y * 0.5 * ws.y);
        out.z = this.clampSize(size.z * 0.5 * ws.z);
        return out;
    }
}
