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

import { getRapierWorld } from './rapier-access';

/**
 * World-level solver tuning, none of which the Cocos physics spec can express.
 *
 * These write live `RAPIER.World` state, so they must be re-applied after
 * `selector.switchTo()` rebuilds the world.
 *
 * Two asymmetries are inherited from Rapier 0.20 rather than invented here: `contactErp`
 * is read-only, and `contactNaturalFrequency` is write-only. The older `erp` property no
 * longer exists at all.
 */
export const rapierSolver = {
    /** Solver iterations per step. Higher is more accurate and slower. Default 4. */
    get numSolverIterations (): number {
        return getRapierWorld()?.numSolverIterations ?? 0;
    },
    set numSolverIterations (v: number) {
        const w = getRapierWorld();
        if (w) w.numSolverIterations = v;
    },

    /** Internal PGS iterations inside each solver iteration. Default 1. */
    get numInternalPgsIterations (): number {
        return getRapierWorld()?.numInternalPgsIterations ?? 0;
    },
    set numInternalPgsIterations (v: number) {
        const w = getRapierWorld();
        if (w) w.numInternalPgsIterations = v;
    },

    /** Maximum CCD substeps per step. Default 1. */
    get maxCcdSubsteps (): number {
        return getRapierWorld()?.maxCcdSubsteps ?? 0;
    },
    set maxCcdSubsteps (v: number) {
        const w = getRapierWorld();
        if (w) w.maxCcdSubsteps = v;
    },

    /**
     * World units per metre. Rapier's defaults assume 1 unit = 1 metre; set this when a
     * project works at a different scale rather than fighting the solver tolerances.
     */
    get lengthUnit (): number {
        return getRapierWorld()?.lengthUnit ?? 1;
    },
    set lengthUnit (v: number) {
        const w = getRapierWorld();
        if (w) w.lengthUnit = v;
    },

    /** Allowed linear penetration, normalized by `lengthUnit`. Default 0.001. */
    get normalizedAllowedLinearError (): number {
        return getRapierWorld()?.integrationParameters.normalizedAllowedLinearError ?? 0;
    },
    set normalizedAllowedLinearError (v: number) {
        const w = getRapierWorld();
        if (w) w.integrationParameters.normalizedAllowedLinearError = v;
    },

    /** Contact prediction distance, normalized by `lengthUnit`. Default 0.002. */
    get normalizedPredictionDistance (): number {
        return getRapierWorld()?.integrationParameters.normalizedPredictionDistance ?? 0;
    },
    set normalizedPredictionDistance (v: number) {
        const w = getRapierWorld();
        if (w) w.integrationParameters.normalizedPredictionDistance = v;
    },

    /** Read-only in Rapier 0.20. Tune stiffness through `contactNaturalFrequency`. */
    get contactErp (): number {
        return getRapierWorld()?.integrationParameters.contact_erp ?? 0;
    },

    /** Write-only in Rapier 0.20; there is no matching getter. */
    set contactNaturalFrequency (v: number) {
        const w = getRapierWorld();
        if (w) w.integrationParameters.contact_natural_frequency = v;
    },
};
