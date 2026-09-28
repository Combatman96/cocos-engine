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
import { BoxCharacterController } from '../../../../exports/physics-framework';
import { IBoxCharacterController } from '../../spec/i-character-controller';
import { RapierCharacterController } from './rapier-character-controller';

/** @mangle */
export class RapierBoxCharacterController extends RapierCharacterController implements IBoxCharacterController {
    get component (): BoxCharacterController {
        return this._comp as BoxCharacterController;
    }

    setHalfHeight (_value: number): void {
        this.updateScale();
    }

    setHalfSideExtent (_value: number): void {
        this.updateScale();
    }

    setHalfForwardExtent (_value: number): void {
        this.updateScale();
    }

    protected buildColliderDesc (): RAPIER.ColliderDesc {
        const comp = this.component;
        const ws = comp.node.worldScale;
        // Side extent runs along X, height along Y, forward extent along Z.
        return this.rapier.ColliderDesc.cuboid(
            Math.max(1e-5, comp.halfSideExtent * Math.abs(ws.x)),
            Math.max(1e-5, comp.halfHeight * Math.abs(ws.y)),
            Math.max(1e-5, comp.halfForwardExtent * Math.abs(ws.z)),
        );
    }
}
