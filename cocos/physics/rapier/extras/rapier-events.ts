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

import { Vec3 } from '../../../core';
import { Collider } from '../../../../exports/physics-framework';

/**
 * Contact-force events. `ICollisionEvent` carries no force data at all, so this is genuinely
 * new capability - useful for destructible objects and impact audio.
 *
 * A collider only produces these once it has `ERapierActiveEvents.CONTACT_FORCE_EVENTS`
 * set (see `setRapierActiveEvents`) and its force exceeds the threshold from
 * `setRapierContactForceEventThreshold`.
 *
 * The underlying `TempContactForceEvent` is a temporary wasm view, so the dispatcher copies
 * every field before calling listeners. The object handed to a listener is reused between
 * events: read it, do not retain it.
 */
export interface IRapierContactForceEvent {
    colliderA: Collider | null;
    colliderB: Collider | null;
    totalForce: Vec3;
    totalForceMagnitude: number;
    maxForceDirection: Vec3;
    maxForceMagnitude: number;
}

export type RapierContactForceCallback = (event: IRapierContactForceEvent) => void;

const listeners: RapierContactForceCallback[] = [];

export const rapierContactForceEvents = {
    on (cb: RapierContactForceCallback): void {
        if (listeners.indexOf(cb) < 0) listeners.push(cb);
    },

    off (cb: RapierContactForceCallback): void {
        const i = listeners.indexOf(cb);
        if (i >= 0) listeners.splice(i, 1);
    },

    /** @internal Called by RapierWorld after each step. */
    _dispatch (event: IRapierContactForceEvent): void {
        for (let i = 0; i < listeners.length; i++) listeners[i](event);
    },

    /** @internal Lets the world skip draining the queue when nobody is listening. */
    _hasListeners (): boolean {
        return listeners.length > 0;
    },
};
