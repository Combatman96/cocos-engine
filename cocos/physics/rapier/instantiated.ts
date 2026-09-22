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

/* eslint-disable import/no-mutable-exports */

import { BUILD, DEBUG, LOAD_RAPIER_MANUALLY } from 'internal:constants';
import type * as RAPIER from '@dimforge/rapier3d-compat';
import { game } from '../../game';
import { error, log, sys } from '../../core';

/**
 * @en
 * The live Rapier module namespace. It is an empty object until
 * `waitForRapierInstantiation()` has resolved.
 *
 * Backend code must import this binding rather than importing
 * '@dimforge/rapier3d-compat' directly, for two reasons:
 * 1. the package is pulled in through a dynamic `import()` below so that the wasm
 *    payload lands in its own rollup chunk instead of the main engine chunk;
 * 2. nothing in the package is usable before `init()` resolves.
 *
 * IMPORTANT: never dereference `R` at module evaluation time. Rapier's enums and
 * classes are only present after instantiation, and every module under
 * `cocos/physics/rapier/` is evaluated long before the boot sequence reaches
 * `Game.EVENT_PRE_SUBSYSTEM_INIT`. Touch `R` from constructors and methods only.
 * @zh
 * Rapier 模块命名空间，在 `waitForRapierInstantiation()` 完成前为空对象。
 * 请勿在模块求值阶段访问 `R`。
 */
export let R = {} as typeof RAPIER;

let rapierReady = false;

/**
 * @en Whether the Rapier wasm module has finished loading.
 * @zh Rapier wasm 模块是否已加载完成。
 */
export function isRapierReady (): boolean {
    return rapierReady;
}

/**
 * @en
 * Throws in debug builds when Rapier is dereferenced before it is ready. This turns an
 * otherwise opaque `Cannot read properties of undefined` into a named failure.
 * @zh
 * 在调试版本中，若 Rapier 尚未就绪便被访问则报错。
 */
export function assertRapierReady (where: string): void {
    if (DEBUG && !rapierReady) {
        error(`[rapier]: ${where} was reached before the rapier wasm module finished loading. `
            + `Await 'loadWasmModuleRapier()' (or 'waitForRapierInstantiation()') first.`);
    }
}

let instantiationPromise: Promise<void> | undefined;

/**
 * @en
 * Loads and initializes the Rapier wasm module. Safe to call repeatedly; the
 * underlying work happens once and every caller awaits the same promise.
 * @zh
 * 加载并初始化 Rapier wasm 模块，可重复调用，实际只会执行一次。
 */
export function waitForRapierInstantiation (): Promise<void> {
    if (instantiationPromise) return instantiationPromise;
    instantiationPromise = (async (): Promise<void> => {
        if (!sys.hasFeature(sys.Feature.WASM)) {
            error('[rapier]: WebAssembly is not supported on this platform, '
                + 'the rapier physics backend is unavailable.');
            return;
        }
        const mod = await import('@dimforge/rapier3d-compat');
        // Tolerate both the ESM namespace and the CommonJS interop shape.
        const ns = (typeof (mod as { init?: unknown }).init === 'function'
            ? mod
            : (mod as unknown as { default: typeof RAPIER }).default) as typeof RAPIER;
        await ns.init();
        R = ns;
        rapierReady = true;
        log(`[rapier]: rapier wasm lib loaded, version ${R.version()}.`);
    })().catch((err: unknown): void => {
        error(`[rapier]: rapier wasm lib load failed: ${err as string}`);
    });
    return instantiationPromise;
}

if (!BUILD || !LOAD_RAPIER_MANUALLY) {
    game.onPostInfrastructureInitDelegate.add(waitForRapierInstantiation);
}
