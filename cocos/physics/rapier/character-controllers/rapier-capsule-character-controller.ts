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
import { CapsuleCharacterController } from '../../../../exports/physics-framework';
import { ICapsuleCharacterController } from '../../spec/i-character-controller';
import { RapierCharacterController } from './rapier-character-controller';

/** @mangle */
export class RapierCapsuleCharacterController extends RapierCharacterController implements ICapsuleCharacterController {
    get component (): CapsuleCharacterController {
        return this._comp as CapsuleCharacterController;
    }

    setRadius (_value: number): void {
        this.updateScale();
    }

    setHeight (_value: number): void {
        this.updateScale();
    }

    protected buildColliderDesc (): RAPIER.ColliderDesc {
        const comp = this.component;
        const ws = comp.node.worldScale;
        const radius = Math.max(1e-5, comp.radius * Math.max(Math.abs(ws.x), Math.abs(ws.z)));
        // The component's `height` is the distance between the two sphere centres, which
        // is exactly twice Rapier's capsule half-height.
        const halfHeight = Math.max(1e-5, comp.height * 0.5 * Math.abs(ws.y));
        return this.rapier.ColliderDesc.capsule(halfHeight, radius);
    }
}
