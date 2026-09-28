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
});
