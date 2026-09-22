/**
 * Declarations for the explicit resource management symbols.
 *
 * TypeScript ships `Symbol.dispose` / `Symbol.asyncDispose` in `lib.esnext.disposable.d.ts`
 * from 5.2 onwards. This repo is pinned to TypeScript 4.9, so the symbols are unknown to
 * the checker.
 *
 * `@dimforge/rapier3d-compat`'s generated wasm-bindgen typings (`dist/rapier_wasm3d.d.ts`)
 * declare `[Symbol.dispose]()` on every wasm-backed class. Without these declarations that
 * file produces ~70 errors, in two flavours:
 *   - TS2339: Property 'dispose' does not exist on type 'SymbolConstructor'
 *   - TS1165: A computed property name in an ambient context must refer to an expression
 *             whose type is a literal type or a 'unique symbol' type
 * Declaring the symbols as `unique symbol` satisfies both.
 *
 * This is deliberately preferred over enabling `skipLibCheck`, which would silence type
 * errors across every dependency in the engine rather than fixing this one gap.
 *
 * Remove this file once the repo moves to TypeScript >= 5.2.
 */

interface SymbolConstructor {
    readonly dispose: unique symbol;
    readonly asyncDispose: unique symbol;
}
