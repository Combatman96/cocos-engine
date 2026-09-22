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
import { IVec3Like, Vec3, warn } from '../../core';
import { PhysicsSystem, RigidBody } from '../../../exports/physics-framework';
import { IRigidBody } from '../spec/i-rigid-body';
import { ERigidBodyType } from '../framework/physics-enum';
import { RapierWorld } from './rapier-world';
import { RapierSharedBody } from './rapier-shared-body';
import { CC_QUAT_0, CC_V3_0, CC_V3_1 } from './rapier-cache';

const v3_0 = new Vec3();
const v3_1 = new Vec3();

let _warnedAboutFactor = false;
let _warnedAboutSleepThreshold = false;

/** @mangle */
export class RapierRigidBody implements IRigidBody {
    private _rigidBody!: RigidBody;
    private _sharedBody!: RapierSharedBody;
    private _isEnabled = false;
    private _sleepThreshold = 0.1;

    get impl (): RAPIER.RigidBody {
        return this._sharedBody.impl;
    }

    get rigidBody (): RigidBody {
        return this._rigidBody;
    }

    get sharedBody (): RapierSharedBody {
        return this._sharedBody;
    }

    get isEnabled (): boolean {
        return this._isEnabled;
    }

    get isAwake (): boolean {
        return !this._sharedBody.impl.isSleeping();
    }

    /**
     * Rapier tracks sleep as a boolean, with no intermediate "about to sleep" state like
     * cannon's AWAKE / SLEEPY / SLEEPING, so this is always false.
     */
    get isSleepy (): boolean {
        return false;
    }

    get isSleeping (): boolean {
        return this._sharedBody.impl.isSleeping();
    }

    initialize (com: RigidBody): void {
        this._rigidBody = com;
        this._sharedBody = (PhysicsSystem.instance.physicsWorld as RapierWorld).getSharedBody(com.node, this);
        this._sharedBody.reference = true;
    }

    onEnable (): void {
        this._isEnabled = true;
        this.setType(this._rigidBody.type);
        this.setMass(this._rigidBody.mass);
        this.setAllowSleep(this._rigidBody.allowSleep);
        this.setLinearDamping(this._rigidBody.linearDamping);
        this.setAngularDamping(this._rigidBody.angularDamping);
        this.setLinearFactor(this._rigidBody.linearFactor);
        this.setAngularFactor(this._rigidBody.angularFactor);
        this.useGravity(this._rigidBody.useGravity);
        this.useCCD(this._rigidBody.useCCD);
        this._sharedBody.enabled = true;
    }

    onDisable (): void {
        this._isEnabled = false;
        this._sharedBody.enabled = false;
    }

    onDestroy (): void {
        this._sharedBody.reference = false;
        (this._rigidBody as unknown) = null;
    }

    setType (v: ERigidBodyType): void {
        this._sharedBody.setType(v);
    }

    setMass (v: number): void {
        this._sharedBody.setMass(v);
    }

    setLinearDamping (v: number): void {
        this._sharedBody.impl.setLinearDamping(v);
    }

    setAngularDamping (v: number): void {
        this._sharedBody.impl.setAngularDamping(v);
    }

    useGravity (v: boolean): void {
        this._sharedBody.impl.setGravityScale(v ? 1 : 0, true);
    }

    /**
     * Cocos models these as per-axis scalar multipliers; Rapier only offers per-axis
     * on/off locks. Any non-zero component therefore becomes "enabled", and a fractional
     * value cannot be honoured.
     */
    setLinearFactor (v: IVec3Like): void {
        this._checkFactor(v);
        this._sharedBody.impl.setEnabledTranslations(v.x !== 0, v.y !== 0, v.z !== 0, true);
    }

    setAngularFactor (v: IVec3Like): void {
        this._checkFactor(v);
        this._sharedBody.impl.setEnabledRotations(v.x !== 0, v.y !== 0, v.z !== 0, true);
    }

    setAllowSleep (v: boolean): void {
        this._sharedBody.setAllowSleep(v);
    }

    wakeUp (): void {
        this._sharedBody.impl.wakeUp();
    }

    sleep (): void {
        this._sharedBody.impl.sleep();
    }

    clearState (): void {
        this.clearVelocity();
        this.clearForces();
    }

    clearForces (): void {
        this._sharedBody.clearForces();
    }

    clearVelocity (): void {
        this._sharedBody.clearVelocity();
    }

    /**
     * Rapier 0.20 does not bind the Rust-side sleep thresholds, so this is stored for
     * round-tripping only. Sleep onset therefore differs from the other backends.
     */
    setSleepThreshold (v: number): void {
        this._sleepThreshold = v;
        if (!_warnedAboutSleepThreshold) {
            _warnedAboutSleepThreshold = true;
            warn('[PHYSICS][rapier]: sleepThreshold is not exposed by the Rapier JS bindings and '
                + 'has no effect. Bodies will fall asleep at Rapier\'s built-in threshold.');
        }
    }

    getSleepThreshold (): number {
        return this._sleepThreshold;
    }

    useCCD (v: boolean): void {
        this._sharedBody.impl.enableCcd(v);
    }

    isUsingCCD (): boolean {
        return this._sharedBody.impl.isCcdEnabled();
    }

    getLinearVelocity (out: IVec3Like): void {
        const v = this._sharedBody.impl.linvel();
        out.x = v.x; out.y = v.y; out.z = v.z;
    }

    setLinearVelocity (value: IVec3Like): void {
        this._sharedBody.impl.setLinvel(value, true);
    }

    getAngularVelocity (out: IVec3Like): void {
        const v = this._sharedBody.impl.angvel();
        out.x = v.x; out.y = v.y; out.z = v.z;
    }

    setAngularVelocity (value: IVec3Like): void {
        this._sharedBody.impl.setAngvel(value, true);
    }

    applyForce (force: IVec3Like, relativePoint?: IVec3Like): void {
        const body = this._sharedBody.impl;
        if (relativePoint) {
            body.addForceAtPoint(force, this._toWorldPoint(relativePoint, false), true);
        } else {
            body.addForce(force, true);
        }
        this._sharedBody.markForcesDirty();
    }

    applyLocalForce (force: IVec3Like, relativePoint?: IVec3Like): void {
        const body = this._sharedBody.impl;
        const worldForce = this._toWorldVector(force);
        if (relativePoint) {
            body.addForceAtPoint(worldForce, this._toWorldPoint(relativePoint, true), true);
        } else {
            body.addForce(worldForce, true);
        }
        this._sharedBody.markForcesDirty();
    }

    applyImpulse (force: IVec3Like, relativePoint?: IVec3Like): void {
        const body = this._sharedBody.impl;
        if (relativePoint) {
            body.applyImpulseAtPoint(force, this._toWorldPoint(relativePoint, false), true);
        } else {
            body.applyImpulse(force, true);
        }
    }

    applyLocalImpulse (force: IVec3Like, relativePoint?: IVec3Like): void {
        const body = this._sharedBody.impl;
        const worldImpulse = this._toWorldVector(force);
        if (relativePoint) {
            body.applyImpulseAtPoint(worldImpulse, this._toWorldPoint(relativePoint, true), true);
        } else {
            body.applyImpulse(worldImpulse, true);
        }
    }

    applyTorque (torque: IVec3Like): void {
        this._sharedBody.impl.addTorque(torque, true);
        this._sharedBody.markForcesDirty();
    }

    applyLocalTorque (torque: IVec3Like): void {
        this._sharedBody.impl.addTorque(this._toWorldVector(torque), true);
        this._sharedBody.markForcesDirty();
    }

    setGroup (v: number): void {
        this._sharedBody.collisionFilterGroup = v;
    }

    getGroup (): number {
        return this._sharedBody.collisionFilterGroup;
    }

    addGroup (v: number): void {
        this._sharedBody.collisionFilterGroup |= v;
    }

    removeGroup (v: number): void {
        this._sharedBody.collisionFilterGroup &= ~v;
    }

    setMask (v: number): void {
        this._sharedBody.collisionFilterMask = v;
    }

    getMask (): number {
        return this._sharedBody.collisionFilterMask;
    }

    addMask (v: number): void {
        this._sharedBody.collisionFilterMask |= v;
    }

    removeMask (v: number): void {
        this._sharedBody.collisionFilterMask &= ~v;
    }

    /**
     * Rapier's `addForceAtPoint`/`applyImpulseAtPoint` take an absolute world point,
     * whereas Cocos passes an offset from the body origin.
     */
    private _toWorldPoint (relativePoint: IVec3Like, isLocal: boolean): IVec3Like {
        const body = this._sharedBody.impl;
        const offset = isLocal ? this._toWorldVector(relativePoint, v3_1) : relativePoint;
        Vec3.add(CC_V3_1, body.translation(CC_V3_0) as Vec3, offset as Vec3);
        return CC_V3_1;
    }

    /** Rotates a body-local vector into world space. */
    private _toWorldVector (v: IVec3Like, out: Vec3 = v3_0): Vec3 {
        // `rotation(target)` fills the target in place and returns it, so CC_QUAT_0 now
        // holds the body's orientation as a real Quat.
        this._sharedBody.impl.rotation(CC_QUAT_0);
        return Vec3.transformQuat(out, v as Vec3, CC_QUAT_0);
    }

    private _checkFactor (v: IVec3Like): void {
        if (_warnedAboutFactor) return;
        const fractional = (n: number): boolean => n !== 0 && n !== 1;
        if (fractional(v.x) || fractional(v.y) || fractional(v.z)) {
            _warnedAboutFactor = true;
            warn('[PHYSICS][rapier]: linearFactor/angularFactor are per-axis on/off locks in '
                + 'Rapier, not scalar multipliers. Fractional components are treated as 1.');
        }
    }
}
