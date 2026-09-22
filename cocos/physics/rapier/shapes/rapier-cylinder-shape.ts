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
import { CylinderCollider } from '../../../../exports/physics-framework';
import { ICylinderShape } from '../../spec/i-physics-shape';
import { EAxisDirection } from '../../framework/physics-enum';
import { RapierAxialShape } from './rapier-axial-shape';
import { axisToRotation } from '../rapier-utils';

/** @mangle */
export class RapierCylinderShape extends RapierAxialShape implements ICylinderShape {
    get collider (): CylinderCollider {
        return this._collider as CylinderCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        axisToRotation(this._rotation, this.collider.direction);
        this._desc = this.rapier.ColliderDesc.cylinder(this.halfHeight(), this.radius());
        this._desc.setRotation(this._rotation);
    }

    protected halfHeight (): number {
        return this.scaledHalfHeight(this.collider.height);
    }

    protected radius (): number {
        return this.scaledRadius(this.collider.radius);
    }

    protected direction (): EAxisDirection {
        return this.collider.direction;
    }
}
