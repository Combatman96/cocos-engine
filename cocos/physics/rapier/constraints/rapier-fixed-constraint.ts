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
import { Quat, Vec3, warn } from '../../../core';
import { FixedConstraint } from '../../../../exports/physics-framework';
import { IFixedConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();
const frameA = new Quat();
const frameB = new Quat();
const invRot = new Quat();

let _warnedAboutBreak = false;

/**
 * Welds two bodies at whatever relative transform they hold when the joint is built.
 *
 * `FixedConstraint` exposes no pivot, so body A's own origin and orientation are used as
 * the joint frame, and body B's frame is that same world transform expressed in B's local
 * space.
 */
/** @mangle */
export class RapierFixedConstraint extends RapierConstraint implements IFixedConstraint {
    get constraint (): FixedConstraint {
        return this._com as FixedConstraint;
    }

    /**
     * Rapier joints do not break. Bullet supports breakForce but treats breakTorque as an
     * accepted no-op, so a partially unsupported constraint is established precedent;
     * here neither has an equivalent.
     */
    setBreakForce (_v: number): void {
        this._warnOnce();
    }

    setBreakTorque (_v: number): void {
        this._warnOnce();
    }

    protected buildJointData (): RAPIER.JointData {
        const node = this._com.node;
        const connected = this._connectedBody;

        Vec3.set(anchorA, 0, 0, 0);
        Quat.identity(frameA);

        if (connected) {
            // Body A's world transform expressed in body B's local frame.
            Quat.invert(invRot, connected.node.worldRotation);
            Vec3.subtract(anchorB, node.worldPosition, connected.node.worldPosition);
            Vec3.transformQuat(anchorB, anchorB, invRot);
            Quat.multiply(frameB, invRot, node.worldRotation);
        } else {
            // The world anchor body sits at the origin with identity rotation, so body A's
            // world transform is already expressed in its frame.
            Vec3.copy(anchorB, node.worldPosition);
            Quat.copy(frameB, node.worldRotation);
        }

        return this.rapier.JointData.fixed(anchorA, frameA, anchorB, frameB);
    }

    private _warnOnce (): void {
        if (_warnedAboutBreak) return;
        _warnedAboutBreak = true;
        warn('[PHYSICS][rapier]: breakForce and breakTorque have no equivalent in Rapier; '
            + 'fixed constraints never break.');
    }
}
