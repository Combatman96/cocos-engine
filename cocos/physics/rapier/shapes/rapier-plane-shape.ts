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

import { IVec3Like, Vec3 } from '../../../core';
import { PlaneCollider } from '../../../../exports/physics-framework';
import { IPlaneShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';

/**
 * A plane's AABB is conceptually infinite. A large finite extent keeps the framework's
 * bounds maths well-behaved while still enclosing anything in a sane scene.
 */
const PLANE_HALF_EXTENT = 1e5;

const offset = new Vec3();
const normal = new Vec3();

/** @mangle */
export class RapierPlaneShape extends RapierShape implements IPlaneShape {
    get collider (): PlaneCollider {
        return this._collider as PlaneCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        // Rapier models an infinite plane as a HalfSpace, and unlike the other primitives
        // there is no `ColliderDesc.halfspace` factory -- the shape is constructed directly.
        this._desc = new this.rapier.ColliderDesc(new this.rapier.HalfSpace(this._normal()));
    }

    setNormal (_v: IVec3Like): void {
        if (this._impl) {
            // A HalfSpace carries its normal inside the shape, so changing the normal means
            // swapping the shape rather than adjusting a parameter.
            this._impl.setShape(new this.rapier.HalfSpace(this._normal()));
        }
        this.setCenter(this._collider.center);
    }

    setConstant (_v: number): void {
        // The constant is a displacement along the normal, so it lives in the collider's
        // local translation.
        this.setCenter(this._collider.center);
    }

    /**
     * Overridden because a plane's placement is `center * scale + normal * constant`,
     * whereas every other shape is positioned by its centre alone.
     */
    setCenter (v: IVec3Like): void {
        Vec3.multiply(offset, v as Vec3, this._collider.node.worldScale);
        Vec3.scaleAndAdd(offset, offset, this._normal(), this.collider.constant);
        this._desc.setTranslation(offset.x, offset.y, offset.z);
        this._desc.setRotation(this._rotation);
        if (this._impl) {
            this._impl.setTranslationWrtParent(offset);
            this._impl.setRotationWrtParent(this._rotation);
        }
    }

    protected updateScale (): void {
        // The normal is scale invariant; only the offset moves.
        this.setCenter(this._collider.center);
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        return out.set(PLANE_HALF_EXTENT, PLANE_HALF_EXTENT, PLANE_HALF_EXTENT);
    }

    protected getLocalBoundingRadius (): number {
        return PLANE_HALF_EXTENT;
    }

    private _normal (): Vec3 {
        Vec3.copy(normal, this.collider.normal);
        // A degenerate normal would make the half-space meaningless; fall back to +Y.
        if (normal.lengthSqr() < 1e-12) Vec3.copy(normal, Vec3.UNIT_Y);
        return Vec3.normalize(normal, normal);
    }
}
