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
import { SphereCollider } from '../../../../exports/physics-framework';
import { ISphereShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';
import { ERapierSharedBodyDirty } from '../rapier-enum';
import { absMaxComponent } from '../rapier-utils';

/** @mangle */
export class RapierSphereShape extends RapierShape implements ISphereShape {
    get collider (): SphereCollider {
        return this._collider as SphereCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._desc = this.rapier.ColliderDesc.ball(this._radius());
    }

    updateRadius (): void {
        if (this._impl) this._impl.setRadius(this._radius());
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    protected updateScale (): void {
        this.updateRadius();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        const r = this._radius();
        return out.set(r, r, r);
    }

    protected getLocalBoundingRadius (): number {
        return this._radius();
    }

    /**
     * A sphere has a single radius, so a non-uniform node scale cannot be represented
     * exactly. The largest scale component is used, which is the same approximation the
     * cannon backend makes (`maxComponent`).
     */
    private _radius (): number {
        return this.clampSize(this.collider.radius * absMaxComponent(this._collider.node.worldScale));
    }
}
