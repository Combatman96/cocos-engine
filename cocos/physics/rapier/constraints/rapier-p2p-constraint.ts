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
import { IVec3Like, Vec3 } from '../../../core';
import { PointToPointConstraint } from '../../../../exports/physics-framework';
import { IPointToPointConstraint } from '../../spec/i-physics-constraint';
import { RapierConstraint } from './rapier-constraint';

const anchorA = new Vec3();
const anchorB = new Vec3();

/** @mangle */
export class RapierP2PConstraint extends RapierConstraint implements IPointToPointConstraint {
    get constraint (): PointToPointConstraint {
        return this._com as PointToPointConstraint;
    }

    setPivotA (_v: IVec3Like): void {
        this._applyAnchors();
    }

    setPivotB (_v: IVec3Like): void {
        this._applyAnchors();
    }

    protected buildJointData (): RAPIER.JointData {
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        return this.rapier.JointData.spherical(anchorA, anchorB);
    }

    /** Anchors are the one part of a spherical joint Rapier can change in place. */
    private _applyAnchors (): void {
        if (!this._impl) return;
        const cs = this.constraint;
        this.scaledPivotA(anchorA, cs.pivotA);
        this.scaledPivotB(anchorB, cs.pivotA, cs.pivotB);
        this._impl.setAnchor1(anchorA);
        this._impl.setAnchor2(anchorB);
    }
}
