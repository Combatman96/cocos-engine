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
import { EAxisDirection } from '../../framework/physics-enum';
import { RapierShape } from './rapier-shape';
import { ERapierSharedBodyDirty } from '../rapier-enum';
import { absMaxPerpendicular, axisToRotation, scaleAlongAxis } from '../rapier-utils';

/**
 * Shared base for the shapes Rapier only builds along its local +Y axis: capsule,
 * cylinder and cone.
 *
 * A Cocos `direction` of X or Z therefore becomes a rotation offset on the collider
 * relative to its parent body, rather than a different shape. Note this is the inverse of
 * the PhysX backend, whose capsule geometry is X-aligned.
 */
/** @mangle */
export abstract class RapierAxialShape extends RapierShape {
    /** Half the length of the shape's central axis, excluding any end caps. */
    protected abstract halfHeight (): number;

    /** Radius perpendicular to the axis. */
    protected abstract radius (): number;

    /** The shape's configured axis. */
    protected abstract direction (): EAxisDirection;

    /** Extra extent the end caps add along the axis (capsules only). */
    protected capExtent (): number {
        return 0;
    }

    setDirection (v: number): void {
        axisToRotation(this._rotation, v as EAxisDirection);
        // The rotation lives on the collider's local transform, so re-pushing the centre
        // is what actually applies it.
        this.setCenter(this._collider.center);
        this.updateGeometry();
    }

    setRadius (_v: number): void {
        this.updateGeometry();
    }

    setHeight (_v: number): void {
        this.updateGeometry();
    }

    setCylinderHeight (_v: number): void {
        this.updateGeometry();
    }

    /** Pushes the current radius and half-height onto the live collider. */
    protected updateGeometry (): void {
        if (this._impl) {
            this._impl.setRadius(this.radius());
            this._impl.setHalfHeight(this.halfHeight());
        }
        this._sharedBody.dirty |= ERapierSharedBodyDirty.MASS_PROPERTIES;
    }

    protected updateScale (): void {
        this.updateGeometry();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        const r = this.radius();
        const h = this.halfHeight() + this.capExtent();
        switch (this.direction()) {
        case EAxisDirection.X_AXIS:
            return out.set(h, r, r);
        case EAxisDirection.Z_AXIS:
            return out.set(r, r, h);
        case EAxisDirection.Y_AXIS:
        default:
            return out.set(r, h, r);
        }
    }

    protected getLocalBoundingRadius (): number {
        return this.halfHeight() + this.capExtent() + this.radius();
    }

    /** Radius scaled by the larger of the two world-scale components across the axis. */
    protected scaledRadius (baseRadius: number): number {
        const ws = this._collider.node.worldScale;
        return this.clampSize(baseRadius * absMaxPerpendicular(ws, this.direction()));
    }

    /** Half-height scaled by the world-scale component along the axis. */
    protected scaledHalfHeight (baseHeight: number): number {
        const ws = this._collider.node.worldScale;
        return Math.max(0, Math.abs(baseHeight) * 0.5 * scaleAlongAxis(ws, this.direction()));
    }
}
