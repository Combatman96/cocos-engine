import { director, game, Game } from "../../cocos/game";
import { physics, PhysicsSystem } from "../../exports/physics-framework";
import "../../exports/physics-rapier";
import "../../exports/physics-cannon";
import { waitForRapierInstantiation } from "../../cocos/physics/rapier/instantiated";
import { RapierCache } from "../../cocos/physics/rapier/rapier-cache";
import { Node, Scene } from "../../cocos/scene-graph";
import { geometry, Quat, Vec3 } from "../../cocos/core";

beforeAll(async () => {
    await waitForRapierInstantiation();
    game.emit(Game.EVENT_PRE_SUBSYSTEM_INIT);
    PhysicsSystem.constructAndRegister();
});

describe('rapier internals', () => {
    let scene: Scene;

    beforeEach(() => {
        physics.selector.switchTo('rapier');
        scene = new Scene('rapier-internals');
        director.runSceneImmediate(scene);
    });

    afterEach(() => {
        scene.destroy();
    });

    test('world.destroy() then component teardown does not double-destroy', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        node.addComponent(physics.RigidBody);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // Destroys every shared body directly, bypassing the ref count.
        PhysicsSystem.instance.physicsWorld.destroy();
        // Component teardown then drives the ref count to zero a second time.
        // `destroy()` is deferred, so a tick is what actually runs the teardown.
        node.destroy();
        expect(() => { director.tick(PhysicsSystem.instance.fixedTimeStep); }).not.toThrow();
    });

    test('step sets the timestep even with no rigid bodies', () => {
        const world = PhysicsSystem.instance.physicsWorld as any;
        world.impl.timestep = 0.5;
        world.step(1 / 90);
        expect(world.impl.timestep).toBeCloseTo(1 / 90, 6);
    });

    test('destroying a collider deregisters its handle', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const shape = (collider as any)._shape;
        const handle = shape.impl.handle as number;

        expect(RapierCache.getShape(handle)).toBe(shape);
        node.destroy();
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        expect(RapierCache.getShape(handle)).toBeUndefined();
    });

    test('allowSleep set before any body exists still reaches later bodies', () => {
        const world = PhysicsSystem.instance.physicsWorld;
        world.setAllowSleep(false);

        // A collider-only node: a RigidBody component would push its own `allowSleep`
        // down in onEnable, which legitimately overrides the world default.
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const sharedBody = (collider as any)._shape.sharedBody;
        expect(sharedBody.allowSleep).toBe(false);
        world.setAllowSleep(true);
    });

    test('debugDrawConstraintSize round-trips', () => {
        const world = PhysicsSystem.instance.physicsWorld;
        world.debugDrawConstraintSize = 0.75;
        expect(world.debugDrawConstraintSize).toBe(0.75);
    });
    test('sweep with a zero-length direction returns no hit and no NaN', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // `geometry.Ray.d` is not guaranteed to be unit length; a zero vector would make
        // Vec3.normalize yield NaN that Rapier propagates into time_of_impact.
        const ray = new geometry.Ray(0, 0, 0, 0, 0, 0);
        const hit = PhysicsSystem.instance.sweepBoxClosest(ray, new Vec3(0.5, 0.5, 0.5), new Quat());
        expect(hit).toBe(false);
        expect(Number.isNaN(PhysicsSystem.instance.sweepCastClosestResult.distance)).toBe(false);
    });

    test('point-to-point constraint binds to the world anchor body when connectedBody is null', () => {
        const node = new Node('anchored');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.RigidBody.Type.DYNAMIC;
        const c = node.addComponent(physics.PointToPointConstraint) as physics.PointToPointConstraint;
        c.pivotA = new Vec3(0, 1, 0);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // Rapier's createImpulseJoint needs two real bodies and has no getFixedBody()
        // equivalent, so a null connectedBody must resolve to the world's anchor body.
        const impl = (c as any)._constraint.impl;
        expect(impl).not.toBeNull();
        expect(impl.isValid()).toBe(true);
    });

    test('rebinding connectedBody does not leak the previous joint', () => {
        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.RigidBody.Type.DYNAMIC;
        rbB.type = physics.RigidBody.Type.DYNAMIC;
        const c = a.addComponent(physics.PointToPointConstraint) as physics.PointToPointConstraint;
        const dt = PhysicsSystem.instance.fixedTimeStep;
        director.tick(dt);

        const world = (PhysicsSystem.instance.physicsWorld as any).impl;
        const baseline = world.impulseJoints.len();

        // null -> body -> null. Each rebind must remove the old joint before making a new
        // one, otherwise the joint set grows every time.
        c.connectedBody = rbB;
        director.tick(dt);
        c.connectedBody = null;
        director.tick(dt);

        expect(world.impulseJoints.len()).toBe(baseline);
    });

    test('hinge constraint converts degree limits to radians', () => {
        const node = new Node('hinge');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.RigidBody.Type.DYNAMIC;
        const h = node.addComponent(physics.HingeConstraint) as physics.HingeConstraint;
        h.axis = new Vec3(0, 1, 0);
        h.limitEnabled = true;
        h.lowerLimit = -90;
        h.upperLimit = 90;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // Cocos components carry degrees; Rapier joints take radians.
        const impl = (h as any)._constraint.impl;
        expect(impl).not.toBeNull();
        expect(impl.limitsMin()).toBeCloseTo(-Math.PI / 2, 5);
        expect(impl.limitsMax()).toBeCloseTo(Math.PI / 2, 5);
    });

    test('fixed constraint holds a dynamic body against a static one', () => {
        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.worldPosition = new Vec3(0, 5, 0);
        b.worldPosition = new Vec3(1, 5, 0);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.RigidBody.Type.STATIC;
        rbB.type = physics.RigidBody.Type.DYNAMIC;
        const f = a.addComponent(physics.FixedConstraint) as physics.FixedConstraint;
        f.connectedBody = rbB;

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 60; i++) director.tick(dt);

        // Welded to a static body, so b must not fall away under gravity. Without the
        // joint a free body drops ~5 units in one second at g = -10.
        expect(b.worldPosition.y).toBeGreaterThan(4.5);
    });

    test('configurable constraint with all linear axes locked holds a body up', () => {
        const a = new Node('a');
        const b = new Node('b');
        scene.addChild(a);
        scene.addChild(b);
        a.worldPosition = new Vec3(0, 5, 0);
        b.worldPosition = new Vec3(1, 5, 0);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        const rbA = a.addComponent(physics.RigidBody) as physics.RigidBody;
        const rbB = b.addComponent(physics.RigidBody) as physics.RigidBody;
        rbA.type = physics.RigidBody.Type.STATIC;
        rbB.type = physics.RigidBody.Type.DYNAMIC;

        const c = a.addComponent(physics.ConfigurableConstraint) as physics.ConfigurableConstraint;
        const ll = c.linearLimitSettings;
        ll.xMotion = physics.EConstraintMode.LOCKED;
        ll.yMotion = physics.EConstraintMode.LOCKED;
        ll.zMotion = physics.EConstraintMode.LOCKED;
        c.connectedBody = rbB;

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 60; i++) director.tick(dt);

        // Every linear axis is locked against a static body, so b cannot fall. A free body
        // drops ~5 units in one second at g = -10.
        expect(b.worldPosition.y).toBeGreaterThan(4.5);
    });

    test('character controller ignores sub-threshold moves and a zero timestep', () => {
        const node = new Node('cct');
        scene.addChild(node);
        const cct = node.addComponent(physics.CapsuleCharacterController) as physics.CapsuleCharacterController;
        cct.minMoveDistance = 0.001;
        cct.centerWorldPosition = new Vec3(0, 10, 0);

        // Shorter than minMoveDistance, so it must not move at all.
        cct.move(new Vec3(0, 0.0001, 0));
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        expect(Vec3.equals(cct.centerWorldPosition as Vec3, new Vec3(0, 10, 0))).toBe(true);

        // elapsedTime is unused by a purely geometric sweep, so zero must be harmless.
        const impl = (cct as any)._cct;
        expect(() => { impl.move(new Vec3(0, 1, 0), 0.001, 0); }).not.toThrow();
        expect(Number.isNaN(cct.centerWorldPosition.y)).toBe(false);
    });

    test('character controller emits onControllerColliderHit when it hits a collider', () => {
        const floor = new Node('floor');
        scene.addChild(floor);
        floor.worldPosition = new Vec3(0, 0, 0);
        const box = floor.addComponent(physics.BoxCollider) as physics.BoxCollider;
        box.size = new Vec3(10, 1, 10);

        const node = new Node('cct');
        scene.addChild(node);
        const cct = node.addComponent(physics.CapsuleCharacterController) as physics.CapsuleCharacterController;
        cct.centerWorldPosition = new Vec3(0, 3, 0);

        let hits = 0;
        let hitCollider: unknown = null;
        cct.on('onControllerColliderHit', (contact: any) => {
            hits++;
            hitCollider = contact.collider;
        });

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 30; i++) {
            cct.move(new Vec3(0, -0.5, 0));
            director.tick(dt);
        }

        expect(hits).toBeGreaterThan(0);
        expect(hitCollider).toBe(box);
    });

    test('non-square terrain lays heights out column-major', () => {
        const sizeI = 3;
        const sizeJ = 5;
        // Height depends only on j, so a transposed buffer is immediately visible.
        const asset = {
            _uuid: 'test-terrain',
            tileSize: 1,
            getVertexCountI: (): number => sizeI,
            getVertexCountJ: (): number => sizeJ,
            getHeight: (_i: number, j: number): number => j,
        };

        const node = new Node('terrain');
        scene.addChild(node);
        const tc = node.addComponent(physics.TerrainCollider) as physics.TerrainCollider;
        tc.terrain = asset as any;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const shape = (tc as any)._shape;
        expect(shape.impl).not.toBeNull();
        // Rapier's heightfield buffer is column-major: index = j * sizeI + i.
        expect(shape.heights.length).toBe(sizeI * sizeJ);
        expect(shape.heights[2 * sizeI + 0]).toBeCloseTo(2, 5);
        expect(shape.heights[4 * sizeI + 1]).toBeCloseTo(4, 5);
        expect(shape.heights[0 * sizeI + 2]).toBeCloseTo(0, 5);
    });

    test('simplex collider builds a convex hull from its vertex count', () => {
        const node = new Node('simplex');
        scene.addChild(node);
        const sc = node.addComponent(physics.SimplexCollider) as physics.SimplexCollider;
        sc.shapeType = 4; // TETRAHEDRON
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        expect((sc as any)._shape.impl).not.toBeNull();
    });

    test('debug draw reads line geometry and tolerates having no camera', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const world = PhysicsSystem.instance.physicsWorld as any;
        world.debugDrawFlags = physics.EPhysicsDrawFlags.WIRE_FRAME;

        // Rapier renders the whole world in one call: a flat line list of 3 floats per
        // vertex, 2 vertices per line, with one RGBA colour per vertex.
        const buffers = world.impl.debugRender();
        expect(buffers.vertices.length).toBeGreaterThan(0);
        expect(buffers.colors.length).toBeGreaterThan(0);

        // jsdom has no camera, so the renderer is unavailable and this must be a no-op
        // rather than a crash.
        expect(() => { world._debugDraw(); }).not.toThrow();

        world.debugDrawFlags = physics.EPhysicsDrawFlags.NONE;
    });

    test('editor exposes rapier as a web-only physics feature with i18n keys', () => {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const fs = require('fs') as typeof import('fs');
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const config = require('../../editor/engine-features/render-config.json');

        const option = config.features.physics.options['physics-rapier'];
        expect(option).toBeDefined();
        expect(option.label).toBe('i18n:ENGINE.features.physics_rapier.label');
        // Web-only: must not be advertised as a native module, and must not steal the
        // default from physics-ammo.
        expect(option.isNativeModule).toBeUndefined();
        expect(option.cmakeConfig).toBeUndefined();
        expect(option.default).toBeUndefined();
        expect(option.flags.LOAD_RAPIER_MANUALLY).toBeDefined();

        for (const locale of ['en', 'zh']) {
            const src = fs.readFileSync(`editor/i18n/${locale}/localization.js`, 'utf8');
            expect(src).toContain('physics_rapier');
            expect(src).toContain('loadWasmModuleRapier');
        }
    });

    test('an empty physicsEngine setting selects the sole custom backend and pins it', () => {
        // Cocos Creator's editor build bundles every backend and writes physicsEngine: ""
        // for a physics feature it does not recognise, then follows up with
        // switchTo('builtin'). Both halves must be handled or editor preview silently
        // runs the dynamics-free builtin backend.
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const { settings, SettingsCategory } = require('../../cocos/core');
        const previous = settings.querySettings(SettingsCategory.PHYSICS, 'physicsEngine');
        settings.overrideSettings(SettingsCategory.PHYSICS, 'physicsEngine', '');
        physics.selector.switchTo('cannon.js');
        expect(physics.selector.id).toBe('cannon.js');
        try {
            (PhysicsSystem as any).selectConfiguredBackend();
            expect(physics.selector.id).toBe('rapier');
            expect(physics.selector.pinnedId).toBe('rapier');

            physics.selector.switchTo('rapier');
            const world = physics.selector.physicsWorld;
            physics.selector.switchTo('builtin');
            expect(physics.selector.id).toBe('rapier');
            // Coerced call must not rebuild the live world either.
            expect(physics.selector.physicsWorld).toBe(world);
        } finally {
            physics.selector.pinnedId = null;
            settings.overrideSettings(SettingsCategory.PHYSICS, 'physicsEngine', previous);
        }
    });
});
