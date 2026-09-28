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
import { Collider, RigidBody } from '../../../../exports/physics-framework';
import { RapierCache } from '../rapier-cache';
import { getRapierCollider, getRapierRigidBody, getRapierWorld } from './rapier-access';

/** Query filtering richer than `IRaycastOptions`, which offers only a mask and a flag. */
export interface IRapierQueryFilter {
    /** A combination of `ERapierQueryFilterFlags`. */
    flags?: number;
    /** A packed Rapier `InteractionGroups` value. */
    groups?: number;
    excludeCollider?: Collider;
    excludeRigidBody?: RigidBody;
    /** Arbitrary per-collider predicate; nothing in the Cocos spec can express this. */
    predicate?: (collider: Collider | null) => boolean;
}

export interface IRapierPointProjection {
    collider: Collider;
    point: Vec3;
    isInside: boolean;
}

function wrapPredicate (filter?: IRapierQueryFilter): ((c: RAPIER.Collider) => boolean) | undefined {
    if (!filter || !filter.predicate) return undefined;
    const predicate = filter.predicate;
    return (c: RAPIER.Collider): boolean => {
        const shape = RapierCache.getShape(c.handle);
        return predicate(shape ? shape.collider : null);
    };
}

function excludeCollider (filter?: IRapierQueryFilter): RAPIER.Collider | undefined {
    if (!filter || !filter.excludeCollider) return undefined;
    return getRapierCollider(filter.excludeCollider) ?? undefined;
}

function excludeBody (filter?: IRapierQueryFilter): RAPIER.RigidBody | undefined {
    if (!filter || !filter.excludeRigidBody) return undefined;
    return getRapierRigidBody(filter.excludeRigidBody) ?? undefined;
}

/** Finds the closest point on any collider to `point`. No Cocos spec equivalent. */
export function rapierProjectPoint (
    point: IVec3Like,
    solid: boolean,
    filter?: IRapierQueryFilter,
): IRapierPointProjection | null {
    const world = getRapierWorld();
    if (!world) return null;
    const hit = world.projectPoint(
        point as RAPIER.Vector,
        solid,
        filter?.flags as RAPIER.QueryFilterFlags | undefined,
        filter?.groups,
        excludeCollider(filter),
        excludeBody(filter),
        wrapPredicate(filter),
    );
    if (!hit) return null;
    const shape = RapierCache.getShape(hit.collider.handle);
    if (!shape || !shape.collider) return null;
    return {
        collider: shape.collider,
        point: new Vec3(hit.point.x, hit.point.y, hit.point.z),
        isInside: hit.isInside,
    };
}

/**
 * Enumerates colliders whose broad-phase AABB overlaps the given box. Return false from
 * the callback to stop early.
 */
export function rapierCollidersInAabb (
    center: IVec3Like,
    halfExtents: IVec3Like,
    callback: (collider: Collider) => boolean,
): void {
    const world = getRapierWorld();
    if (!world) return;
    world.collidersWithAabbIntersectingAabb(
        center as RAPIER.Vector,
        halfExtents as RAPIER.Vector,
        (c): boolean => {
            const shape = RapierCache.getShape(c.handle);
            if (!shape || !shape.collider) return true;
            return callback(shape.collider);
        },
    );
}

/** Walks the contact manifolds between two colliders, if they are touching. */
export function rapierForEachContactManifold (
    a: Collider,
    b: Collider,
    callback: (manifold: RAPIER.TempContactManifold, flipped: boolean) => void,
): void {
    const world = getRapierWorld();
    const ca = getRapierCollider(a);
    const cb = getRapierCollider(b);
    if (!world || !ca || !cb) return;
    // The manifold is a reused view freed after the callback; copy anything you keep.
    world.contactPair(ca, cb, callback);
}
