const { pathsToModuleNameMapper } = require('ts-jest');
const ts = require('typescript');
const ps = require('path');
const tsConfig = ts.readConfigFile(ps.join(__dirname, 'tsconfig.json'), ts.sys.readFile);
if (!tsConfig.config) {
    throw new Error(`Failed to read tsconfig`);
}
const { compilerOptions } = tsConfig.config;
const tsPathsMapper = pathsToModuleNameMapper(compilerOptions.paths, { prefix: `${__dirname}/` });
for (const key of Object.keys(tsPathsMapper)) {
    if (key.includes('rapier3d')) delete tsPathsMapper[key];
}

module.exports = {
    testEnvironment: './tests/test-environment.ts',
    testRegex: '/tests/.*\\.(test|spec)?\\.(ts|tsx)$',
    moduleNameMapper: {
        ...tsPathsMapper,
        // tsconfig paths point this at the ESM build for the engine bundlers, but jest runs
        // CommonJS and does not transform node_modules, so it must take the CJS build.
        '^@dimforge/rapier3d-compat$': '<rootDir>/node_modules/@dimforge/rapier3d-compat/dist/rapier.cjs',
        'external:(.*)': '<rootDir>/native/external/$1',
    },
    transformIgnorePatterns: [
        // ignore everything in the node_modules EXCEPT for:
        // - @cocos/dragonbones-js
        'node_modules/(?!(@cocos/dragonbones-js)/)',
        // ignore everything in the native/external/emscripten EXCEPT for:
        // - meshopt
        // Since above packages are in ESM module format, whereas we currently use CJS for testing.
        'native/external/emscripten/(?!(meshopt)/)',
    ],
    setupFilesAfterEnv: [
        "./tests/setup-after-env.ts",
        "./tests/utils/log-capture-setup-after-env.ts"
    ],
    setupFiles: [
        './tests/init.ts',
    ],
    coverageDirectory: './test/report/',
    globals: {
        CC_DEV: true,
        CC_TEST: true,
        CC_PHYSICS_BUILTIN: true,
    }
};
