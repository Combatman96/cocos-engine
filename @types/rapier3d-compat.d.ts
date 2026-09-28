/**
 * Ambient declaration for the Rapier physics bindings.
 *
 * `scripts/patch-rapier-package.js` deletes the `types` / `typings` fields from the
 * installed package, because Cocos Creator resolves the engine's dependencies with
 * TypeScript's own `resolveModuleName` — which prefers `types` over `main`, `module` and
 * the `exports` map. That made Creator resolve the package to `dist/rapier.d.ts`, hand
 * that declaration file to Rollup as a module to bundle, and fail with:
 *
 *     Could not resolve './exports' from .../dist/rapier.d.ts
 *
 * With `types` gone the specifier resolves to `dist/rapier.cjs`, a real JavaScript file,
 * exactly as `@cocos/cannon` and `@cocos/box2d` do — neither of which declares `types`.
 *
 * Module resolution and type lookup are separate concerns in TypeScript, so this ambient
 * declaration restores the full typings without changing what `resolveModuleName`
 * answers. Do NOT reinstate the `types` field or add a `paths` entry pointing at the
 * `.d.ts`: either one puts a declaration file back into Creator's bundle graph.
 *
 * The re-export goes through the `@cocos/rapier3d-types` tsconfig alias. A relative
 * specifier is illegal inside an ambient module declaration (TS2439), and the package
 * subpath is unresolvable because the patched "exports" map declares only "." — plain
 * `tsc` tolerates that via node10 resolution, but ccbuild's TypeScript plugin does not.
 * The alias is type-only, so it never enters the runtime module graph.
 */
declare module '@cocos/rapier3d-compat' {
    export * from '@cocos/rapier3d-types';
}

/**
 * The Rapier runtime, published under the engine's `external:` origin by
 * `scripts/patch-rapier-package.js`.
 *
 * The runtime import must go through this id rather than the npm package name. Cocos
 * Creator only emits external npm dependencies reached by a STATIC import; a dynamic
 * `import()` of one is left dangling, and SystemJS then fails at runtime with "Unable to
 * resolve bare specifier". ccbuild's externalWasmLoader turns any unmatched `external:`
 * id into a real SystemJS module, which is how the bullet backend loads its wasm, and is
 * the only route that works with a dynamic import inside Creator.
 */
declare module 'external:rapier/rapier.js' {
    export * from '@cocos/rapier3d-types';
}

