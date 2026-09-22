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

import { IVec3Like, Quat, Vec3, warn } from '../../core';
import { EAxisDirection } from '../framework/physics-enum';
import type { IRaycastOptions } from '../spec/i-physics-world';
import { ERapierQueryFilterFlags } from './rapier-enum';

/**
 * Packs a Cocos group/mask pair into Rapier's `InteractionGroups` bitfield.
 *
 * Rapier stores interaction groups as a single 32-bit value laid out as
 * `(membership << 16) | filter`, i.e. **16 bits of membership and 16 bits of filter**.
 * Cocos, by contrast, carries an independent 32-bit group and a 32-bit mask and supports
 * 32 collision groups (`PhysicsSystem.collisionMatrix` is keyed by `1 << i` for i in
 * [0, 32)).
 *
 * The top 16 bits therefore cannot be represented. We fold them down onto the low 16, so
 * group 16 becomes indistinguishable from group 0, group 17 from group 1, and so on. The
 * failure mode is a false *positive*: objects that should not collide may collide. That is
 * deliberately the safer direction — the alternative (dropping the high bits) would make
 * objects silently fall through the world.
 *
 * Projects needing true 32-group fidelity must opt into the `PhysicsHooks`-based exact
 * filter, which costs a JS callback per candidate narrow-phase pair per step.
 */
export function packInteractionGroups (group: number, mask: number): number {
    const g = ((group & 0xFFFF) | ((group >>> 16) & 0xFFFF)) & 0xFFFF;
    const m = ((mask & 0xFFFF) | ((mask >>> 16) & 0xFFFF)) & 0xFFFF;
    return ((g << 16) | m) >>> 0;
}

let _warnedAboutGroupWidth = false;

/**
 * Warns once per session when a project uses a collision group that Rapier's 16-bit
 * membership field cannot represent.
 */
export function checkGroupWidth (value: number): void {
    if (!_warnedAboutGroupWidth && (value >>> 16) !== 0) {
        _warnedAboutGroupWidth = true;
        warn('[PHYSICS][rapier]: collision groups above bit 15 cannot be represented by the '
            + '16-bit interaction groups used by Rapier. Group N and group N+16 will be treated '
            + 'as the same group, which may cause unintended collisions. Use groups 0-15, or '
            + 'enable exact collision filtering.');
    }
}

/**
 * Rapier's capsule, cylinder and cone shapes are built along the local **+Y** axis only,
 * so a Cocos `direction` of X or Z becomes a rotation offset on the collider relative to
 * its parent body.
 *
 * Note this is the opposite of the PhysX backend, whose capsule geometry is X-aligned —
 * do not copy the Euler angles from `physx-capsule-shape.ts`, they are inverted here.
 */
export function axisToRotation (out: Quat, dir: EAxisDirection): Quat {
    switch (dir) {
    case EAxisDirection.X_AXIS:
        return Quat.rotationTo(out, Vec3.UNIT_Y, Vec3.UNIT_X);
    case EAxisDirection.Z_AXIS:
        return Quat.rotationTo(out, Vec3.UNIT_Y, Vec3.UNIT_Z);
    case EAxisDirection.Y_AXIS:
    default:
        return Quat.copy(out, Quat.IDENTITY);
    }
}

/**
 * The largest absolute component of `v`, used to collapse a non-uniform node scale onto a
 * single radius. Matches the approximation the cannon backend makes.
 */
export function absMaxComponent (v: IVec3Like): number {
    return Math.max(Math.abs(v.x), Math.max(Math.abs(v.y), Math.abs(v.z)));
}

/**
 * The larger of the two absolute scale components perpendicular to `dir`. Used for the
 * radius of capsules, cylinders and cones, matching `bullet-capsule-shape.ts`.
 */
export function absMaxPerpendicular (v: IVec3Like, dir: EAxisDirection): number {
    switch (dir) {
    case EAxisDirection.X_AXIS:
        return Math.max(Math.abs(v.y), Math.abs(v.z));
    case EAxisDirection.Z_AXIS:
        return Math.max(Math.abs(v.x), Math.abs(v.y));
    case EAxisDirection.Y_AXIS:
    default:
        return Math.max(Math.abs(v.x), Math.abs(v.z));
    }
}

/** The scale component along `dir`, used for the half-height of capsules/cylinders/cones. */
export function scaleAlongAxis (v: IVec3Like, dir: EAxisDirection): number {
    switch (dir) {
    case EAxisDirection.X_AXIS:
        return Math.abs(v.x);
    case EAxisDirection.Z_AXIS:
        return Math.abs(v.z);
    case EAxisDirection.Y_AXIS:
    default:
        return Math.abs(v.y);
    }
}

/**
 * Translates the Cocos raycast options into Rapier query filter flags.
 * Rapier calls sensors out explicitly, so `queryTrigger: false` becomes EXCLUDE_SENSORS.
 */
export function toQueryFilterFlags (options: IRaycastOptions): number {
    return options.queryTrigger ? 0 : ERapierQueryFilterFlags.EXCLUDE_SENSORS;
}

/**
 * Builds the `InteractionGroups` value for a scene query.
 *
 * Membership is all-ones so that every collider's filter bits accept the query, and the
 * filter half carries the query's mask so we only hit colliders whose membership
 * intersects it.
 *
 * `IRaycastOptions.group` is intentionally ignored, matching the other backends: bullet
 * passes only `options.mask`, and cannon hardcodes `collisionFilterGroup: -1`.
 */
export function toQueryGroups (options: IRaycastOptions): number {
    const m = ((options.mask & 0xFFFF) | ((options.mask >>> 16) & 0xFFFF)) & 0xFFFF;
    return ((0xFFFF << 16) | m) >>> 0;
}
