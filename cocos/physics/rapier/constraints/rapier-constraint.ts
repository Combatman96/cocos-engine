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
import { Constraint, PhysicsSystem, RigidBody } from '../../../../exports/physics-framework';
import { IBaseConstraint } from '../../spec/i-physics-constraint';
import { RapierWorld } from '../rapier-world';
import { RapierRigidBody } from '../rapier-rigid-body';
import { R } from '../instantiated';

/** @mangle */
export abstract class RapierConstraint implements IBaseConstraint {
    protected _com!: Constraint;
    protected _rigidBody!: RigidBody;
    protected _connectedBody: RigidBody | null = null;
    protected _collided = false;
    protected _impl: RAPIER.ImpulseJoint | null = null;
    protected _world!: RapierWorld;
    private _needRebuild = false;
    private _enabled = false;

    get impl (): RAPIER.ImpulseJoint | null {
        return this._impl;
    }

    get constraint (): Constraint {
        return this._com;
    }

    /** Builds the descriptor describing the joint for the component's current state. */
    protected abstract buildJointData (): RAPIER.JointData;

    initialize (v: Constraint): void {
        this._com = v;
        this._rigidBody = v.attachedBody!;
        this._connectedBody = v.connectedBody;
        this._collided = v.enableCollision;
        this._world = PhysicsSystem.instance.physicsWorld as RapierWorld;
    }

    onEnable (): void {
        this._enabled = true;
        this._world.addConstraint(this);
        this.createJoint();
    }

    onDisable (): void {
        this._enabled = false;
        this.destroyJoint();
        this._world.removeConstraint(this);
    }

    onDestroy (): void {
        this.destroyJoint();
        this._world.removeConstraint(this);
        (this._com as unknown) = null;
        (this._rigidBody as unknown) = null;
        this._connectedBody = null;
    }

    setConnectedBody (v: RigidBody | null): void {
        if (this._connectedBody === v) return;
        this._connectedBody = v;
        this.scheduleRebuild();
    }

    setEnableCollision (v: boolean): void {
        if (this._collided === v) return;
        this._collided = v;
        if (this._impl) this._impl.setContactsEnabled(v);
    }

    /**
     * Most of Rapier's joint state lives on the descriptor rather than the joint, so a
     * changed property means the joint has to be recreated. Rebuilds are deferred to the
     * next `syncSceneToPhysics` so that setting N properties in one frame costs one
     * rebuild rather than N.
     */
    protected scheduleRebuild (): void {
        this._needRebuild = true;
    }

    flushRebuild (): void {
        if (!this._needRebuild) return;
        this._needRebuild = false;
        if (!this._enabled) return;
        this.destroyJoint();
        this.createJoint();
    }

    createJoint (): void {
        if (this._impl) return;
        const wrapped = this._rigidBody ? this._rigidBody.body as RapierRigidBody | null : null;
        if (!wrapped) return;
        const bodyA = wrapped.sharedBody.impl;
        const connected = this._connectedBody;
        // Rapier's `createImpulseJoint` needs two real bodies and offers no equivalent of
        // Bullet's shared fixed body, so a null connectedBody anchors to the world's.
        const bodyB = connected && connected.body
            ? (connected.body as RapierRigidBody).sharedBody.impl
            : this._world.anchorBody;
        this._impl = this._world.impl.createImpulseJoint(this.buildJointData(), bodyA, bodyB, true);
        this._impl.setContactsEnabled(this._collided);
    }

    destroyJoint (): void {
        if (!this._impl) return;
        this._world.impl.removeImpulseJoint(this._impl, true);
        this._impl = null;
    }

    /** Pivot scaled by the owning node's world scale, matching every other backend. */
    protected scaledPivotA (out: Vec3, pivot: IVec3Like): Vec3 {
        return Vec3.multiply(out, pivot, this._com.node.worldScale);
    }

    /**
     * With no connected body the second anchor is body A's pivot expressed in world space,
     * because the anchor body sits at the origin with identity rotation.
     */
    protected scaledPivotB (out: Vec3, pivotA: IVec3Like, pivotB: IVec3Like): Vec3 {
        const connected = this._connectedBody;
        if (connected) {
            return Vec3.multiply(out, pivotB, connected.node.worldScale);
        }
        const node = this._com.node;
        Vec3.multiply(out, pivotA, node.worldScale);
        Vec3.transformQuat(out, out, node.worldRotation);
        return Vec3.add(out, out, node.worldPosition);
    }

    protected get rapier (): typeof R {
        return R;
    }
}
