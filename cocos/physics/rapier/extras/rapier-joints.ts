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
import { IVec3Like, Vec3 } from '../../../core';
import { RigidBody } from '../../../../exports/physics-framework';
import { getRapier, getRapierRigidBody, getRapierWorld } from './rapier-access';

const ORIGIN: IVec3Like = new Vec3();

/** Opaque handle returned by the joint creators; pass it back to `destroyRapierJoint`. */
export interface IRapierJointHandle {
    readonly impl: RAPIER.ImpulseJoint;
}

export interface IRapierSpringOptions {
    restLength: number;
    stiffness: number;
    damping: number;
    anchorA?: IVec3Like;
    anchorB?: IVec3Like;
}

export interface IRapierRopeOptions {
    length: number;
    anchorA?: IVec3Like;
    anchorB?: IVec3Like;
}

export interface IRapierGenericOptions {
    /** A combination of `ERapierJointAxesMask` naming the LOCKED axes. */
    axesMask: number;
    axis?: IVec3Like;
    anchorA?: IVec3Like;
    anchorB?: IVec3Like;
}

function create (data: RAPIER.JointData, a: RigidBody, b: RigidBody): IRapierJointHandle | null {
    const world = getRapierWorld();
    const bodyA = getRapierRigidBody(a);
    const bodyB = getRapierRigidBody(b);
    if (!world || !bodyA || !bodyB) return null;
    return { impl: world.createImpulseJoint(data, bodyA, bodyB, true) };
}

/** A distance spring. No Cocos constraint type corresponds to this. */
export function createRapierSpringJoint (
    a: RigidBody,
    b: RigidBody,
    options: IRapierSpringOptions,
): IRapierJointHandle | null {
    const R = getRapier();
    if (!R) return null;
    const data = R.JointData.spring(
        options.restLength,
        options.stiffness,
        options.damping,
        (options.anchorA ?? ORIGIN) as RAPIER.Vector,
        (options.anchorB ?? ORIGIN) as RAPIER.Vector,
    );
    return create(data, a, b);
}

/** A maximum-distance rope. No Cocos constraint type corresponds to this either. */
export function createRapierRopeJoint (
    a: RigidBody,
    b: RigidBody,
    options: IRapierRopeOptions,
): IRapierJointHandle | null {
    const R = getRapier();
    if (!R) return null;
    const data = R.JointData.rope(
        options.length,
        (options.anchorA ?? ORIGIN) as RAPIER.Vector,
        (options.anchorB ?? ORIGIN) as RAPIER.Vector,
    );
    return create(data, a, b);
}

/** A 6-DOF joint with an explicit locked-axis mask, without a Cocos component. */
export function createRapierGenericJoint (
    a: RigidBody,
    b: RigidBody,
    options: IRapierGenericOptions,
): IRapierJointHandle | null {
    const R = getRapier();
    if (!R) return null;
    const data = R.JointData.generic(
        (options.anchorA ?? ORIGIN) as RAPIER.Vector,
        (options.anchorB ?? ORIGIN) as RAPIER.Vector,
        (options.axis ?? Vec3.UNIT_Y) as RAPIER.Vector,
        options.axesMask as RAPIER.JointAxesMask,
    );
    return create(data, a, b);
}

/**
 * A reduced-coordinate joint. Multibody joints do not drift the way impulse joints can,
 * which suits articulated chains such as robot arms and vehicle suspension, at the cost of
 * exposing no limits or motors at all.
 */
export function createRapierMultibodyFixedJoint (
    a: RigidBody,
    b: RigidBody,
    anchorA?: IVec3Like,
    anchorB?: IVec3Like,
): RAPIER.MultibodyJoint | null {
    const R = getRapier();
    const world = getRapierWorld();
    const bodyA = getRapierRigidBody(a);
    const bodyB = getRapierRigidBody(b);
    if (!R || !world || !bodyA || !bodyB) return null;
    const identity = { x: 0, y: 0, z: 0, w: 1 } as RAPIER.Rotation;
    const data = R.JointData.fixed(
        (anchorA ?? ORIGIN) as RAPIER.Vector,
        identity,
        (anchorB ?? ORIGIN) as RAPIER.Vector,
        identity,
    );
    return world.createMultibodyJoint(data, bodyA, bodyB, true);
}

export function destroyRapierJoint (handle: IRapierJointHandle | null): void {
    const world = getRapierWorld();
    if (!world || !handle) return;
    world.removeImpulseJoint(handle.impl, true);
}
