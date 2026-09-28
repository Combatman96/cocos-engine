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
import { IVec3Like, Vec3, toRadian } from '../../../core';
import { HingeConstraint } from '../../../../exports/physics-framework';
import { IHingeConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();
const axis = new Vec3();

/**
 * Hinge over `JointData.revolute`.
 *
 * The revolute joint is the only type whose `RevoluteImpulseJoint` extends
 * `UnitImpulseJoint`, so it is the one constraint where limits and the motor can be driven
 * live instead of through a descriptor rebuild.
 */
/** @mangle */
export class RapierHingeConstraint extends RapierConstraint implements IHingeConstraint {
    get constraint (): HingeConstraint {
        return this._com as HingeConstraint;
    }

    private get unitJoint (): RAPIER.RevoluteImpulseJoint | null {
        return this._impl as RAPIER.RevoluteImpulseJoint | null;
    }

    setPivotA (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setPivotB (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setAxis (_v: IVec3Like): void {
        // The axis is baked into the descriptor, so it cannot be changed in place.
        this.scheduleRebuild();
    }

    setLimitEnabled (v: boolean): void {
        // setLimits() enables limits but there is no call to switch them back off, so
        // disabling means rebuilding: a fresh revolute joint starts unlimited.
        if (!v) {
            this.scheduleRebuild();
            return;
        }
        if (this.unitJoint) this._applyLimits();
        else this.scheduleRebuild();
    }

    setLowerLimit (_min: number): void {
        this._applyLimits();
    }

    setUpperLimit (_max: number): void {
        this._applyLimits();
    }

    setMotorEnabled (v: boolean): void {
        const joint = this.unitJoint;
        if (!joint) return;
        const cs = this.constraint;
        if (v) {
            joint.configureMotorVelocity(toRadian(cs.motorVelocity), 1);
            joint.setMotorMaxForce(cs.motorForceLimit);
        } else {
            joint.configureMotorVelocity(0, 0);
            joint.setMotorMaxForce(0);
        }
    }

    setMotorVelocity (v: number): void {
        const joint = this.unitJoint;
        if (!joint || !this.constraint.motorEnabled) return;
        joint.configureMotorVelocity(toRadian(v), 1);
    }

    setMotorForceLimit (v: number): void {
        const joint = this.unitJoint;
        if (!joint || !this.constraint.motorEnabled) return;
        joint.setMotorMaxForce(v);
    }

    createJoint (): void {
        super.createJoint();
        // Limits and the motor both live on the joint rather than the descriptor, so they
        // have to be re-applied every time the joint is built. A freshly created revolute
        // joint starts unlimited, which is exactly the disabled-limit state.
        this._applyLimits();
        this.setMotorEnabled(this.constraint.motorEnabled);
    }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        Vec3.normalize(axis, cs.axis);
        if (Vec3.lengthSqr(axis) < 1e-12) Vec3.copy(axis, Vec3.UNIT_Y);
        // NOTE: JointData.limitsEnabled / .limits are silently ignored for revolute joints
        // in Rapier 0.20 - only RevoluteImpulseJoint.setLimits() takes effect. Limits are
        // therefore applied in createJoint() once the joint object exists.
        return this.rapier.JointData.revolute(anchorA, anchorB, axis);
    }

    private _applyAnchors (): void {
        if (!this._impl) return;
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        this._impl.setAnchor1(anchorA);
        this._impl.setAnchor2(anchorB);
    }

    private _applyLimits (): void {
        const joint = this.unitJoint;
        const cs = this.constraint;
        if (!joint || !cs.limitEnabled) return;
        joint.setLimits(toRadian(cs.lowerLimit), toRadian(cs.upperLimit));
    }
}
