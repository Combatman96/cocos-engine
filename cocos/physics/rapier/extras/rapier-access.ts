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
import { CharacterController, Collider, PhysicsSystem, RigidBody } from '../../../../exports/physics-framework';
import { selector } from '../../framework/physics-selector';
import { R, isRapierReady } from '../instantiated';
import type { RapierWorld } from '../rapier-world';
import type { RapierRigidBody } from '../rapier-rigid-body';
import type { RapierShape } from '../shapes/rapier-shape';
import type { RapierCharacterController } from '../character-controllers/rapier-character-controller';

/*
 * Typed access to the live Rapier objects.
 *
 * `IPhysicsWorld.impl`, `IRigidBody.impl` and `IBaseShape.impl` already expose them, but
 * as `any`. These accessors give the same reach with real types, and each returns null
 * when rapier is not the active backend so calling code can feature-detect instead of
 * guessing.
 */

/** Whether rapier is the active backend AND its wasm module has finished loading. */
export function isRapierActive (): boolean {
    return selector.id === 'rapier' && isRapierReady();
}

/**
 * The whole RAPIER namespace: enums, `ColliderDesc`, `JointData`, and everything this
 * module does not wrap. Null before `init()` resolves.
 */
export function getRapier (): typeof RAPIER | null {
    return isRapierReady() ? R : null;
}

export function getRapierWorld (): RAPIER.World | null {
    if (!isRapierActive()) return null;
    const world = PhysicsSystem.instance.physicsWorld as RapierWorld | null;
    if (!world || world.destroyed) return null;
    return world.impl;
}

export function getRapierRigidBody (body: RigidBody): RAPIER.RigidBody | null {
    if (!isRapierActive() || !body.body) return null;
    return (body.body as RapierRigidBody).sharedBody.impl;
}

export function getRapierCollider (collider: Collider): RAPIER.Collider | null {
    if (!isRapierActive() || !collider.shape) return null;
    return (collider.shape as unknown as RapierShape).impl;
}

export function getRapierCharacterController (cct: CharacterController): RAPIER.KinematicCharacterController | null {
    if (!isRapierActive()) return null;
    const impl = (cct as unknown as { _cct: RapierCharacterController | null })._cct;
    return impl ? impl.impl : null;
}
