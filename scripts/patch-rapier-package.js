/**
 * Prepares the Rapier physics dependency for the Cocos engine build.
 *
 * Two independent problems are solved here, both discovered against Cocos Creator 3.8.8.
 *
 * 1. COMPILE. Creator resolves the engine's external npm dependencies to a file and hands
 *    that file to Rollup. Rapier ships a "types" field, which made Creator select
 *    dist/rapier.d.ts as the module to bundle and fail with
 *    "Could not resolve './exports' from .../dist/rapier.d.ts". @cocos/cannon and
 *    @cocos/box2d declare no "types" at all, which is why they have always worked.
 *    Dropping "types" makes the specifier resolve to real JavaScript; the typings are
 *    restored by the ambient declaration in @types/rapier3d-compat.d.ts.
 *
 * 2. RUNTIME. Creator only emits external npm dependencies that are reached by a STATIC
 *    import. A dynamic import() of one survives into the output with nothing to resolve
 *    it, and SystemJS then fails with "Unable to resolve bare specifier" — or, for a
 *    virtual module id, "Only absolute URLs are supported". Copying the ESM build under
 *    native/external/ lets the engine load it through the `external:` origin instead,
 *    which ccbuild's externalWasmLoader turns into a real SystemJS module. That is the
 *    same mechanism the bullet backend uses for its wasm, and it is the only route that
 *    works with a dynamic import inside Creator.
 *
 * native/external/ is gitignored, so this runs from postinstall on every install.
 */

const fs = require('fs');
const path = require('path');

const packageRoot = path.join(__dirname, '..', 'node_modules', '@cocos', 'rapier3d-compat');
const packageJsonPath = path.join(packageRoot, 'package.json');

if (!fs.existsSync(packageJsonPath)) {
    throw new Error(`Rapier package is not installed: ${packageJsonPath}`);
}

// --- 1. drop the "types" field so Creator resolves the package to JavaScript -----------
const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
delete packageJson.types;
delete packageJson.typings;
packageJson.exports = {
    '.': {
        import: './dist/rapier.mjs',
        require: './dist/rapier.cjs',
    },
};
fs.writeFileSync(packageJsonPath, `${JSON.stringify(packageJson, null, 2)}\n`);

// --- 2. publish the ESM build under the engine's external: origin ----------------------
const source = path.join(packageRoot, 'dist', 'rapier.mjs');
if (!fs.existsSync(source)) {
    throw new Error(`Rapier ESM build is missing: ${source}`);
}

const targetDir = path.join(__dirname, '..', 'native', 'external', 'rapier');
const target = path.join(targetDir, 'rapier.js');
fs.mkdirSync(targetDir, { recursive: true });
fs.copyFileSync(source, target);

console.log(`[rapier] package prepared, ESM build copied to ${path.relative(path.join(__dirname, '..'), target)}`);
