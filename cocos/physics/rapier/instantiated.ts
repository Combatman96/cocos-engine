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
import type * as RAPIER from '@cocos/rapier3d-compat';
import { game } from '../../game';
import { error, log, sys } from '../../core';

/**
 * @en
 * The live Rapier module namespace. It is an empty object until
 * `waitForRapierInstantiation()` has resolved.
 *
 * Backend code must import this binding rather than importing the package directly,
 * because nothing in the package is usable before `init()` resolves.
 *
 * The package is imported STATICALLY, exactly as the cannon backend imports
 * `@cocos/cannon`. Cocos Creator collects an engine's external npm dependencies from its
 * static imports and emits each one into `editor/external/` (`%40cocos/cannon.js` and so
 * on). A dynamic `import()` is not collected, so the specifier survives into the output
 * with nothing to resolve it and the editor fails at runtime with either "Unable to
 * resolve bare specifier" or "Only absolute URLs are supported", depending on whether a
 * package name or a virtual module id was used.
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
        // Loaded through the engine's external: origin, not the npm package name; see
        // @types/rapier3d-compat.d.ts for why that distinction matters in Creator.
        const mod = await import('external:rapier/rapier.js');
        // Tolerate both the ESM namespace and a CommonJS interop shape.
        const ns: typeof RAPIER = typeof (mod as { init?: unknown }).init === 'function'
            ? (mod as unknown as typeof RAPIER)
            : (mod as unknown as { default: typeof RAPIER }).default;
        // The wasm is inlined in the package as base64, so init() needs no URL and never
        // fetches; passing one is what previously produced "Only absolute URLs".
        await ns.init();
        R = ns;
        rapierReady = true;
        log(`[rapier]: rapier wasm lib loaded, version ${R.version()}.`);
    })().catch((err: unknown): void => {
        error(`[rapier]: rapier wasm lib load failed: ${err as string}`);
        // Do not let subsystem initialization continue and construct a RapierWorld with an
        // unavailable module; that would only replace the useful load error with
        // "R.World is not a constructor".
        throw err;
    });
    return instantiationPromise;
}

// Registered unconditionally, exactly as the bullet backend does. Gating this on
// EDITOR_NOT_IN_PREVIEW stops the module loading during Preview in Editor, which fails
// silently: no world is built, nothing falls, and nothing is logged.
if (!BUILD || !LOAD_RAPIER_MANUALLY) {
    game.onPostInfrastructureInitDelegate.add(waitForRapierInstantiation);
}
