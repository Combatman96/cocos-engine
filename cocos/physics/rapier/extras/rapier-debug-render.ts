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
 * Raw debug geometry for the whole world, for projects that want to draw it themselves
 * rather than through `EPhysicsDrawFlags`.
 *
 * The layout is a flat line list: 3 floats per vertex, 2 vertices per line, with one RGBA
 * colour per vertex. The arrays belong to Rapier's debug pipeline and are re-wrapped on
 * each call, so copy them if they need to outlive the current step.
 */
export function rapierDebugRenderBuffers (): { vertices: Float32Array; colors: Float32Array } | null {
    const world = getRapierWorld();
    if (!world) return null;
    const buffers = world.debugRender();
    return { vertices: buffers.vertices, colors: buffers.colors };
}
