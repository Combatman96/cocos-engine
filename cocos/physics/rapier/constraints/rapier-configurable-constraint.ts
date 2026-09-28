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
import { IVec3Like, Vec3, warn } from '../../../core';
import { ConfigurableConstraint } from '../../../../exports/physics-framework';
import { EConstraintMode, EDriverMode } from '../../framework/physics-enum';
import { IConfigurableConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';
import { RAPIER_AXIS_TO_MASK } from '../rapier-enum';

const anchorA = new Vec3();
const anchorB = new Vec3();
const axis = new Vec3();

let _warnedAboutLimits = false;
let _warnedAboutDrivers = false;

/**
 * Six-degree-of-freedom constraint over Rapier's generic joint.
 *
 * `JointData.generic` accepts only a bitmask of *locked* axes, and the resulting
 * `GenericImpulseJoint` exposes no members at all - no limits, no motors, no runtime
 * mutation of any kind. So:
 *
 * - `EConstraintMode.LOCKED` and `FREE` are honoured exactly.
 * - `EConstraintMode.LIMITED` degrades to FREE, with a one-time warning.
 * - Every driver mode degrades to DISABLED, with a one-time warning.
 * - Soft-constraint stiffness, damping and restitution have no effect.
 *
 * Because the mask is descriptor-only, every mode change schedules a rebuild rather than
 * mutating the live joint.
 */
/** @mangle */
export class RapierConfigurableConstraint extends RapierConstraint implements IConfigurableConstraint {
    private _lockedMask = 0;

    get constraint (): ConfigurableConstraint {
        return this._com as ConfigurableConstraint;
    }

    setConstraintMode (idx: number, v: EConstraintMode): void {
        const bit = RAPIER_AXIS_TO_MASK[idx];
        if (bit === undefined) return;
        if (v === EConstraintMode.LOCKED) {
            this._lockedMask |= bit;
        } else {
            this._lockedMask &= ~bit;
            if (v === EConstraintMode.LIMITED) this._warnLimits();
        }
        this.scheduleRebuild();
    }

    setLinearLimit (_idx: number, _lower: number, _upper: number): void { this._warnLimits(); }
    setAngularExtent (_twist: number, _swing1: number, _swing2: number): void { this._warnLimits(); }
    setLinearRestitution (_v: number): void { this._warnLimits(); }
    setSwingRestitution (_v: number): void { this._warnLimits(); }
    setTwistRestitution (_v: number): void { this._warnLimits(); }
    setLinearSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setLinearStiffness (_v: number): void { this._warnLimits(); }
    setLinearDamping (_v: number): void { this._warnLimits(); }
    setSwingSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setTwistSoftConstraint (_v: boolean): void { this._warnLimits(); }
    setSwingStiffness (_v: number): void { this._warnLimits(); }
    setSwingDamping (_v: number): void { this._warnLimits(); }
    setTwistStiffness (_v: number): void { this._warnLimits(); }
    setTwistDamping (_v: number): void { this._warnLimits(); }

    setDriverMode (_idx: number, v: EDriverMode): void {
        if (v !== EDriverMode.DISABLED) this._warnDrivers();
    }

    setLinearMotorTarget (_v: IVec3Like): void { this._warnDrivers(); }
    setLinearMotorVelocity (_v: IVec3Like): void { this._warnDrivers(); }
    setLinearMotorForceLimit (_v: number): void { this._warnDrivers(); }
    setAngularMotorTarget (_v: IVec3Like): void { this._warnDrivers(); }
    setAngularMotorVelocity (_v: IVec3Like): void { this._warnDrivers(); }
    setAngularMotorForceLimit (_v: number): void { this._warnDrivers(); }

    setPivotA (_v: IVec3Like): void { this.scheduleRebuild(); }
    setPivotB (_v: IVec3Like): void { this.scheduleRebuild(); }
    setAutoPivotB (_v: boolean): void { this.scheduleRebuild(); }
    setAxis (_v: IVec3Like): void { this.scheduleRebuild(); }
    setSecondaryAxis (_v: IVec3Like): void { this.scheduleRebuild(); }

    setBreakForce (_v: number): void { /* Rapier joints do not break. */ }
    setBreakTorque (_v: number): void { /* Rapier joints do not break. */ }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        if (cs.autoPivotB) {
            // Auto pivot B means "wherever body A's pivot currently is in world space".
            this.scaledPivotB(anchorB, cs.pivotA, cs.pivotA);
        } else {
            this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        }
        Vec3.normalize(axis, cs.axis);
        if (Vec3.lengthSqr(axis) < 1e-12) Vec3.copy(axis, Vec3.UNIT_Y);
        return this.rapier.JointData.generic(anchorA, anchorB, axis, this._lockedMask as RAPIER.JointAxesMask);
    }

    private _warnLimits (): void {
        if (_warnedAboutLimits) return;
        _warnedAboutLimits = true;
        warn('[PHYSICS][rapier]: the generic joint supports only free and locked axes. '
            + 'LIMITED axes behave as FREE, and limit softness, stiffness, damping and '
            + 'restitution have no effect.');
    }

    private _warnDrivers (): void {
        if (_warnedAboutDrivers) return;
        _warnedAboutDrivers = true;
        warn('[PHYSICS][rapier]: the generic joint exposes no motors, so '
            + 'ConfigurableConstraint drivers have no effect.');
    }
}
