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
import { Vec3, warn } from '../../../core';
import { Mesh } from '../../../3d/assets';
import { MeshCollider } from '../../../../exports/physics-framework';
import { ITrimeshShape } from '../../spec/i-physics-shape';
import { RapierShape } from './rapier-shape';
import { ERapierTriMeshFlags } from '../rapier-enum';

/**
 * Concave triangle meshes get adjacency-aware contact normals, which removes most of the
 * spurious "ghost collision" bumps a naive trimesh floor produces, plus degenerate
 * triangle removal.
 */
const TRIMESH_FLAGS = ERapierTriMeshFlags.FIX_INTERNAL_EDGES | ERapierTriMeshFlags.DELETE_DEGENERATE_TRIANGLES;

/** @mangle */
export class RapierTrimeshShape extends RapierShape implements ITrimeshShape {
    /** Unscaled source geometry, concatenated across submeshes. */
    private _srcPositions: Float32Array | null = null;
    private _srcIndices: Uint32Array | null = null;
    /** Scale baked copy handed to Rapier. */
    private _scaledPositions: Float32Array | null = null;
    private _localHalfExtents = new Vec3();
    private _localRadius = 0;

    get collider (): MeshCollider {
        return this._collider as MeshCollider;
    }

    protected onComponentSet (): void {
        Vec3.copy(this._bakedScale, this._collider.node.worldScale);
        this._readMesh(this.collider.mesh);
        this._desc = this._buildDesc();
    }

    setMesh (_v: Mesh | null): void {
        this._readMesh(this.collider.mesh);
        this._desc = this._buildDesc();
        // Geometry cannot be swapped on a live collider, so the collider is rebuilt. That
        // also mints a new handle, which `rebuildCollider` re-registers.
        this.rebuildCollider();
    }

    /**
     * Rapier colliders carry no scale, so the node's scale has to be baked into the vertex
     * buffer and the BVH re-cooked. This is O(vertices) plus a rebuild, unlike every
     * primitive shape which resizes in place.
     *
     * Animating the scale of a MeshCollider under this backend therefore costs a full
     * rebuild per change; prefer swapping meshes, or use a primitive/convex proxy.
     */
    protected updateScale (): void {
        this._desc = this._buildDesc();
        this.rebuildCollider();
    }

    protected getLocalHalfExtents (out: Vec3): Vec3 {
        return Vec3.copy(out, this._localHalfExtents);
    }

    protected getLocalBoundingRadius (): number {
        return this._localRadius;
    }

    /**
     * Reads positions and indices out of the mesh, concatenating every rendering submesh
     * so a multi-material mesh does not silently lose geometry.
     *
     * Indices are widened to Uint32Array because that is what Rapier requires. Note this
     * is a fidelity improvement over the cannon backend, which narrows to Uint16Array and
     * so corrupts meshes with more than 65535 vertices.
     */
    private _readMesh (mesh: Mesh | null): void {
        this._srcPositions = null;
        this._srcIndices = null;
        this._scaledPositions = null;
        if (!mesh) return;

        const subMeshes = mesh.renderingSubMeshes;
        let vertexCount = 0;
        let indexCount = 0;
        for (let i = 0; i < subMeshes.length; i++) {
            const info = subMeshes[i].geometricInfo;
            if (!info || !info.positions) continue;
            vertexCount += info.positions.length;
            if (info.indices) indexCount += info.indices.length;
        }
        if (vertexCount === 0 || indexCount === 0) {
            warn(`[PHYSICS][rapier]: mesh '${mesh.name}' has no readable geometry. `
                + 'Enable "Calculate Geometry Info" on the mesh asset.');
            return;
        }

        const positions = new Float32Array(vertexCount);
        const indices = new Uint32Array(indexCount);
        let vOffset = 0;
        let iOffset = 0;
        for (let i = 0; i < subMeshes.length; i++) {
            const info = subMeshes[i].geometricInfo;
            if (!info || !info.positions || !info.indices) continue;
            positions.set(info.positions, vOffset);
            const src = info.indices;
            // Indices are submesh-local, so shift them by the vertices already written.
            const base = vOffset / 3;
            for (let k = 0; k < src.length; k++) indices[iOffset + k] = src[k] + base;
            vOffset += info.positions.length;
            iOffset += src.length;
        }
        this._srcPositions = positions;
        this._srcIndices = indices;
    }

    /** Bakes the current world scale into a fresh vertex buffer and builds the desc. */
    private _buildDesc (): RAPIER.ColliderDesc {
        const src = this._srcPositions;
        const indices = this._srcIndices;
        if (!src || !indices) return this._emptyDesc();

        const ws = this._collider.node.worldScale;
        const scaled = this._scaledPositions && this._scaledPositions.length === src.length
            ? this._scaledPositions
            : new Float32Array(src.length);
        for (let i = 0; i < src.length; i += 3) {
            scaled[i] = src[i] * ws.x;
            scaled[i + 1] = src[i + 1] * ws.y;
            scaled[i + 2] = src[i + 2] * ws.z;
        }
        this._scaledPositions = scaled;
        this._computeLocalBounds(scaled);

        if (this.collider.convex) {
            const desc = this.rapier.ColliderDesc.convexHull(scaled);
            if (desc) return desc;
            warn('[PHYSICS][rapier]: convex hull construction failed for mesh collider on '
                + `'${this._collider.node.name}' (degenerate geometry). Falling back to a trimesh.`);
        }
        return this.rapier.ColliderDesc.trimesh(scaled, indices, TRIMESH_FLAGS);
    }

    /** A tiny degenerate shape, so a collider with no usable mesh is still well-formed. */
    private _emptyDesc (): RAPIER.ColliderDesc {
        const s = this.clampSize(0);
        this._localHalfExtents.set(s, s, s);
        this._localRadius = s;
        return this.rapier.ColliderDesc.cuboid(s, s, s);
    }

    private _computeLocalBounds (positions: Float32Array): void {
        let maxX = 0;
        let maxY = 0;
        let maxZ = 0;
        let maxSqr = 0;
        for (let i = 0; i < positions.length; i += 3) {
            const x = Math.abs(positions[i]);
            const y = Math.abs(positions[i + 1]);
            const z = Math.abs(positions[i + 2]);
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            if (z > maxZ) maxZ = z;
            const sqr = x * x + y * y + z * z;
            if (sqr > maxSqr) maxSqr = sqr;
        }
        this._localHalfExtents.set(maxX, maxY, maxZ);
        this._localRadius = Math.sqrt(maxSqr);
    }
}
