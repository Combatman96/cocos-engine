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
import { IVec3Like } from '../../../core';
import { CharacterController } from '../../../../exports/physics-framework';
import { getRapierCharacterController } from './rapier-access';

export interface IRapierAutostep {
    maxHeight: number;
    minWidth: number;
    includeDynamicBodies: boolean;
}

/**
 * The tuning knobs Rapier's kinematic character controller exposes that the Cocos
 * `CharacterController` component does not - autostep, snap-to-ground, slide behaviour,
 * slope angles and impulse transfer.
 *
 * A cohesive set on one entity, hence a wrapper object rather than free functions.
 * Angles are radians, matching Rapier. Nullable properties map onto Rapier's
 * enable/disable pairs: null means disabled.
 */
export class RapierCharacterTuning {
    constructor (private readonly _impl: RAPIER.KinematicCharacterController) {}

    get autostep (): IRapierAutostep | null {
        if (!this._impl.autostepEnabled()) return null;
        return {
            maxHeight: this._impl.autostepMaxHeight() ?? 0,
            minWidth: this._impl.autostepMinWidth() ?? 0,
            includeDynamicBodies: this._impl.autostepIncludesDynamicBodies() ?? false,
        };
    }

    set autostep (v: IRapierAutostep | null) {
        if (!v) this._impl.disableAutostep();
        else this._impl.enableAutostep(v.maxHeight, v.minWidth, v.includeDynamicBodies);
    }

    get snapToGroundDistance (): number | null {
        return this._impl.snapToGroundEnabled() ? this._impl.snapToGroundDistance() : null;
    }

    set snapToGroundDistance (v: number | null) {
        if (v === null) this._impl.disableSnapToGround();
        else this._impl.enableSnapToGround(v);
    }

    /** Radians. Mirrors `CharacterController.slopeLimit`, which is degrees. */
    get maxSlopeClimbAngle (): number {
        return this._impl.maxSlopeClimbAngle();
    }

    set maxSlopeClimbAngle (v: number) {
        this._impl.setMaxSlopeClimbAngle(v);
    }

    /** Radians. Below this the character slides rather than standing. */
    get minSlopeSlideAngle (): number {
        return this._impl.minSlopeSlideAngle();
    }

    set minSlopeSlideAngle (v: number) {
        this._impl.setMinSlopeSlideAngle(v);
    }

    get applyImpulsesToDynamicBodies (): boolean {
        return this._impl.applyImpulsesToDynamicBodies();
    }

    set applyImpulsesToDynamicBodies (v: boolean) {
        this._impl.setApplyImpulsesToDynamicBodies(v);
    }

    /** Null means the mass is taken from the character's own collider. */
    get characterMass (): number | null {
        return this._impl.characterMass();
    }

    set characterMass (v: number | null) {
        this._impl.setCharacterMass(v);
    }

    /** Whether the character slides along obstacles instead of stopping dead. */
    get slideEnabled (): boolean {
        return this._impl.slideEnabled();
    }

    set slideEnabled (v: boolean) {
        this._impl.setSlideEnabled(v);
    }

    get normalNudgeFactor (): number {
        return this._impl.normalNudgeFactor();
    }

    set normalNudgeFactor (v: number) {
        this._impl.setNormalNudgeFactor(v);
    }

    /** The controller's skin. Mirrors `CharacterController.skinWidth`. */
    get offset (): number {
        return this._impl.offset();
    }

    set offset (v: number) {
        this._impl.setOffset(v);
    }

    /** The up axis used for grounding and slope tests. */
    setUp (v: IVec3Like): void {
        this._impl.setUp(v as RAPIER.Vector);
    }

    get grounded (): boolean {
        return this._impl.computedGrounded();
    }
}

export function getRapierCharacterTuning (cct: CharacterController): RapierCharacterTuning | null {
    const impl = getRapierCharacterController(cct);
    return impl ? new RapierCharacterTuning(impl) : null;
}
