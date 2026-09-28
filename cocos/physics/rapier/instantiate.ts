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

import { BUILD, LOAD_RAPIER_MANUALLY } from 'internal:constants';
import { Game, game } from '../../game';
import { selector } from '../framework/physics-selector';
import { RapierWorld } from './rapier-world';
import { RapierRigidBody } from './rapier-rigid-body';
import { RapierBoxShape } from './shapes/rapier-box-shape';
import { RapierSphereShape } from './shapes/rapier-sphere-shape';
import { RapierCapsuleShape } from './shapes/rapier-capsule-shape';
import { RapierCylinderShape } from './shapes/rapier-cylinder-shape';
import { RapierConeShape } from './shapes/rapier-cone-shape';
import { RapierTrimeshShape } from './shapes/rapier-trimesh-shape';
import { RapierPlaneShape } from './shapes/rapier-plane-shape';
import { RapierP2PConstraint } from './constraints/rapier-p2p-constraint';
import { waitForRapierInstantiation } from './instantiated';
import { PhysicsSystem } from '../framework';

/*
 * Wrapper slots deliberately left unregistered for now:
 *   TerrainShape, SimplexShape, three of the four constraints, and both character
 *   controllers.
 *
 * `check()` in physics-selector.ts turns each missing slot into a
 * "rapier physics does not support X" warning plus a no-op stub, so a partial backend is
 * a supported state rather than a crash. They are filled in as the port progresses.
 */
/*
 * Registration is unconditional and must stay that way. It only stores class references,
 * so it is safe before the wasm module exists — and it has to happen here because
 * EVENT_PRE_SUBSYSTEM_INIT is a one-shot that fires long before `loadWasmModuleRapier()`
 * would run in the manual-load path. Gating it on `isRapierReady()` would mean the
 * backend never registers at all when that flag is set.
 *
 * If the module genuinely fails to load, `RapierWorld`'s constructor reports it by name
 * via `assertRapierReady` rather than failing with an opaque property access.
 */
game.once(Game.EVENT_PRE_SUBSYSTEM_INIT, () => {
    selector.register('rapier', {
        PhysicsWorld: RapierWorld,
        RigidBody: RapierRigidBody,

        BoxShape: RapierBoxShape,
        SphereShape: RapierSphereShape,
        CapsuleShape: RapierCapsuleShape,
        CylinderShape: RapierCylinderShape,
        ConeShape: RapierConeShape,
        TrimeshShape: RapierTrimeshShape,
        PlaneShape: RapierPlaneShape,

        PointToPointConstraint: RapierP2PConstraint,
    });
});

let loadRapierPromise: Promise<void> | undefined;

/**
 * @en
 * Loads the Rapier wasm module and then constructs the physics system. Only needed when
 * the project was built with the "Load Manually" flag for this backend; otherwise the
 * engine loads Rapier during its own boot sequence and this resolves immediately.
 * @zh
 * 手动加载 Rapier wasm 模块并构建物理系统。仅在构建时开启了手动加载选项时才需要调用。
 */
export function loadWasmModuleRapier (): Promise<void> {
    if (BUILD && LOAD_RAPIER_MANUALLY) {
        if (loadRapierPromise) return loadRapierPromise;
        loadRapierPromise = Promise.resolve()
            .then(() => waitForRapierInstantiation())
            .then(() => PhysicsSystem.constructAndRegisterManually());
        return loadRapierPromise;
    }
    return Promise.resolve();
}
