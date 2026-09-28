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
import { IVec3Like, Vec3 } from '../../../core';
import { TerrainCollider } from '../../../../exports/physics-framework';
import { ITerrainShape } from '../../spec/i-physics-shape';
import { ITerrainAsset } from '../../spec/i-external';
import { RapierShape } from './rapier-shape';
import { ERapierHeightFieldFlags } from '../rapier-enum';

const offset = new Vec3();
const extents = new Vec3();

/** @mangle */
export class RapierTerrainShape extends RapierShape implements ITerrainShape {
    /** Column-major height buffer handed to Rapier. */
    heights: Float32Array = new Float32Array(0);

    private _sizeI = 0;
    private _sizeJ = 0;
    private _tileSize = 1;
    private readonly _localOffset = new Vec3();

    get collider (): TerrainCollider {
        return this._collider as TerrainCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._readTerrain(this.collider.terrain);
        this._desc = this._buildDesc();
    }

    setTerrain (v: ITerrainAsset | null): void {
        this._readTerrain(v);
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected updateScale (): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    /**
     * Rapier centres a heightfield on its local origin, whereas a Cocos terrain is
     * anchored at its corner, so the collider is shifted by half the field's extent.
     */
    setCenter (v: IVec3Like): void {
        Vec3.multiply(offset, v as Vec3, this._collider.node.worldScale);
        Vec3.add(offset, offset, this._localOffset);
        this._desc.setTranslation(offset.x, offset.y, offset.z);
        if (this._impl) this._impl.setTranslationWrtParent(offset);
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        const ws = this._collider.node.worldScale;
        return out.set(
            Math.max(1e-5, (this._sizeI - 1) * this._tileSize * 0.5 * Math.abs(ws.x)),
            1e5,
            Math.max(1e-5, (this._sizeJ - 1) * this._tileSize * 0.5 * Math.abs(ws.z)),
        );
    }

    protected getLocalBoundingRadius (): number {
        return this.getLocalHalfExtents(extents).length();
    }

    private _readTerrain (asset: ITerrainAsset | null): void {
        if (!asset) {
            this._sizeI = 0;
            this._sizeJ = 0;
            this.heights = new Float32Array(0);
            return;
        }
        this._tileSize = asset.tileSize;
        this._sizeI = asset.getVertexCountI();
        this._sizeJ = asset.getVertexCountJ();

        // Rapier's nrows/ncols are CELL counts, so the buffer holds
        // (nrows + 1) * (ncols + 1) === sizeI * sizeJ samples, laid out COLUMN-major:
        // index = column * (nrows + 1) + row, i.e. j * sizeI + i. Cocos reads row-major
        // through getHeight(i, j), so this loop is where the transposition happens.
        const heights = new Float32Array(this._sizeI * this._sizeJ);
        for (let j = 0; j < this._sizeJ; j++) {
            for (let i = 0; i < this._sizeI; i++) {
                heights[j * this._sizeI + i] = asset.getHeight(i, j);
            }
        }
        this.heights = heights;
    }

    private _buildDesc (): RAPIER.ColliderDesc {
        if (this._sizeI < 2 || this._sizeJ < 2) {
            this._localOffset.set(0, 0, 0);
            return this.rapier.ColliderDesc.cuboid(1e-5, 1e-5, 1e-5);
        }
        const ws = this._collider.node.worldScale;
        const extentX = (this._sizeI - 1) * this._tileSize * Math.abs(ws.x);
        const extentZ = (this._sizeJ - 1) * this._tileSize * Math.abs(ws.z);
        this._localOffset.set(extentX * 0.5, 0, extentZ * 0.5);

        // `scale` here is the total extent of the field's local x,z plane, not a factor.
        return this.rapier.ColliderDesc.heightfield(
            this._sizeI - 1,
            this._sizeJ - 1,
            this.heights,
            { x: extentX, y: Math.abs(ws.y), z: extentZ },
            ERapierHeightFieldFlags.FIX_INTERNAL_EDGES as RAPIER.HeightFieldFlags,
        );
    }
}
