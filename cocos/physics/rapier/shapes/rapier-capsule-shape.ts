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
import { CapsuleCollider } from '../../../../exports/physics-framework';
import { ICapsuleShape } from '../../spec/i-physics-shape';
import { EAxisDirection } from '../../framework/physics-enum';
import { RapierAxialShape } from './rapier-axial-shape';
import { axisToRotation } from '../rapier-utils';

/** @mangle */
export class RapierCapsuleShape extends RapierAxialShape implements ICapsuleShape {
    get collider (): CapsuleCollider {
        return this._collider as CapsuleCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        axisToRotation(this._rotation, this.collider.direction);
        // Rapier's capsule takes the half-length of the cylindrical section, excluding the
        // hemispherical caps -- the same convention as Cocos's `cylinderHeight`.
        this._desc = this.rapier.ColliderDesc.capsule(this.halfHeight(), this.radius());
        this._desc.setRotation(this._rotation);
    }

    protected halfHeight (): number {
        return this.scaledHalfHeight(this.collider.cylinderHeight);
    }

    protected radius (): number {
        return this.scaledRadius(this.collider.radius);
    }

    protected direction (): EAxisDirection {
        return this.collider.direction;
    }

    /** The caps extend the shape by one radius at each end. */
    protected capExtent (): number {
        return this.radius();
    }
}
