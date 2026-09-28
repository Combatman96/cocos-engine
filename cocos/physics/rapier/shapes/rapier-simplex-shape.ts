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
import { IVec3Like, Vec3, warn } from '../../../core';
import { SimplexCollider } from '../../../../exports/physics-framework';
import { ISimplexShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';

const scratch = new Vec3();

/** @mangle */
export class RapierSimplexShape extends RapierShape implements ISimplexShape {
    private _points = new Float32Array(0);

    get collider (): SimplexCollider {
        return this._collider as SimplexCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._desc = this._buildDesc();
    }

    setShapeType (_v: SimplexCollider.ESimplexType): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    setVertices (_v: IVec3Like[]): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected updateScale (): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        let x = 1e-5;
        let y = 1e-5;
        let z = 1e-5;
        for (let i = 0; i < this._points.length; i += 3) {
            x = Math.max(x, Math.abs(this._points[i]));
            y = Math.max(y, Math.abs(this._points[i + 1]));
            z = Math.max(z, Math.abs(this._points[i + 2]));
        }
        return out.set(x, y, z);
    }

    protected getLocalBoundingRadius (): number {
        return this.getLocalHalfExtents(scratch).length();
    }

    /**
     * `ESimplexType` doubles as the vertex count (POINT = 1 ... TETRAHEDRON = 4), the same
     * trick the bullet backend uses. A convex hull needs four non-coplanar points, so
     * lower counts degenerate to a small ball rather than failing outright.
     */
    private _buildDesc (): RAPIER.ColliderDesc {
        const collider = this.collider;
        const count = collider.shapeType as number;
        const vertices = collider.vertices;
        const ws = collider.node.worldScale;

        const points = new Float32Array(count * 3);
        for (let i = 0; i < count; i++) {
            points[i * 3] = vertices[i].x * ws.x;
            points[i * 3 + 1] = vertices[i].y * ws.y;
            points[i * 3 + 2] = vertices[i].z * ws.z;
        }
        this._points = points;

        if (count >= 4) {
            const hull = this.rapier.ColliderDesc.convexHull(points);
            if (hull) return hull;
            warn(`[PHYSICS][rapier]: degenerate simplex on '${collider.node.name}', falling back to a sphere.`);
        }
        return this.rapier.ColliderDesc.ball(Math.max(1e-5, this.getLocalBoundingRadius()));
    }
}
