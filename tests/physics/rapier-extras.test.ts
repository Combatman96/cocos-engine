import { director, game, Game } from "../../cocos/game";
import { physics, PhysicsSystem } from "../../exports/physics-framework";
import "../../exports/physics-rapier";
import "../../exports/physics-cannon";
import { waitForRapierInstantiation } from "../../cocos/physics/rapier/instantiated";
import {
    getRapier, getRapierWorld, getRapierRigidBody, getRapierCollider, isRapierActive,
    rapierSolver, setRapierCcdEnabled, setRapierDominanceGroup,
    setRapierAdditionalSolverIterations, setRapierContactSkin, setRapierSolverGroups,
    ERapierActiveEvents, ERapierQueryFilterFlags, ERapierActiveHooks, ERapierSolverFlags,
} from "../../cocos/physics/rapier/extras";
import { Node, Scene } from "../../cocos/scene-graph";
import { Vec3 } from "../../cocos/core";

beforeAll(async () => {
    await waitForRapierInstantiation();
    game.emit(Game.EVENT_PRE_SUBSYSTEM_INIT);
    PhysicsSystem.constructAndRegister();
});

describe('rapier extras', () => {
    let scene: Scene;

    beforeEach(() => {
        physics.selector.switchTo('rapier');
        scene = new Scene('rapier-extras');
        director.runSceneImmediate(scene);
    });

    afterEach(() => {
        scene.destroy();
    });

    test('mirrored enums match the real Rapier values', () => {
        const R = getRapier()!;
        expect(ERapierActiveEvents.COLLISION_EVENTS).toBe(R.ActiveEvents.COLLISION_EVENTS);
        expect(ERapierActiveEvents.CONTACT_FORCE_EVENTS).toBe(R.ActiveEvents.CONTACT_FORCE_EVENTS);
        expect(ERapierQueryFilterFlags.EXCLUDE_SENSORS).toBe(R.QueryFilterFlags.EXCLUDE_SENSORS);
        expect(ERapierActiveHooks.FILTER_CONTACT_PAIRS).toBe(R.ActiveHooks.FILTER_CONTACT_PAIRS);
        expect(ERapierSolverFlags.COMPUTE_IMPULSE).toBe(R.SolverFlags.COMPUTE_IMPULSE);
    });

    test('typed accessors resolve the live Rapier objects', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        expect(isRapierActive()).toBe(true);
        expect(getRapierWorld()).toBe((PhysicsSystem.instance.physicsWorld as any).impl);
        expect(getRapierRigidBody(rb)).not.toBeNull();
        expect(getRapierCollider(collider)).not.toBeNull();
    });

    test('accessors return null when rapier is not the active backend', () => {
        physics.selector.switchTo('cannon.js');
        expect(isRapierActive()).toBe(false);
        expect(getRapierWorld()).toBeNull();
    });

    test('solver settings round-trip through the live world', () => {
        rapierSolver.numSolverIterations = 8;
        expect(rapierSolver.numSolverIterations).toBe(8);

        rapierSolver.numInternalPgsIterations = 2;
        expect(rapierSolver.numInternalPgsIterations).toBe(2);

        rapierSolver.lengthUnit = 2;
        expect(rapierSolver.lengthUnit).toBeCloseTo(2, 5);

        // contactErp is read-only in Rapier 0.20; contactNaturalFrequency is write-only.
        expect(typeof rapierSolver.contactErp).toBe('number');
        expect(() => { rapierSolver.contactNaturalFrequency = 30; }).not.toThrow();
    });

    test('per-body tuning reaches Rapier', () => {
        const node = new Node('box');
        scene.addChild(node);
        node.addComponent(physics.BoxCollider);
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.RigidBody.Type.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        setRapierCcdEnabled(rb, true);
        expect(getRapierRigidBody(rb)!.isCcdEnabled()).toBe(true);

        setRapierDominanceGroup(rb, 5);
        expect(getRapierRigidBody(rb)!.dominanceGroup()).toBe(5);

        setRapierAdditionalSolverIterations(rb, 3);
        expect(getRapierRigidBody(rb)!.additionalSolverIterations()).toBe(3);
    });

    test('per-collider tuning reaches Rapier', () => {
        const node = new Node('box');
        scene.addChild(node);
        const collider = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        setRapierContactSkin(collider, 0.05);
        expect(getRapierCollider(collider)!.contactSkin()).toBeCloseTo(0.05, 5);

        setRapierSolverGroups(collider, 0x0003FFFF);
        expect(getRapierCollider(collider)!.solverGroups()).toBe(0x0003FFFF);
    });

    test('physics hooks can veto a contact pair', () => {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const extras = require('../../cocos/physics/rapier/extras');

        const floor = new Node('floor');
        scene.addChild(floor);
        floor.worldPosition = new Vec3(0, 0, 0);
        const floorCollider = floor.addComponent(physics.BoxCollider) as physics.BoxCollider;
        floorCollider.size = new Vec3(20, 1, 20);

        const ball = new Node('ball');
        scene.addChild(ball);
        ball.worldPosition = new Vec3(0, 5, 0);
        const ballCollider = ball.addComponent(physics.SphereCollider) as physics.SphereCollider;
        const rb = ball.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.RigidBody.Type.DYNAMIC;
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        // A hook only fires for colliders that opt in.
        extras.setRapierActiveHooks(ballCollider, extras.ERapierActiveHooks.FILTER_CONTACT_PAIRS);

        let called = false;
        extras.setRapierPhysicsHooks({
            filterContactPair: (): null => { called = true; return null; },
        });

        const dt = PhysicsSystem.instance.fixedTimeStep;
        for (let i = 0; i < 120; i++) director.tick(dt);
        extras.setRapierPhysicsHooks(null);

        expect(called).toBe(true);
        // Every contact was vetoed, so the ball falls straight through the floor.
        expect(ball.worldPosition.y).toBeLessThan(0);
    });

    test('point projection finds the nearest collider', () => {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const { rapierProjectPoint } = require('../../cocos/physics/rapier/extras');

        const node = new Node('box');
        scene.addChild(node);
        node.worldPosition = new Vec3(0, 0, 0);
        node.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const hit = rapierProjectPoint(new Vec3(5, 0, 0), true);
        expect(hit).not.toBeNull();
        expect(hit.collider.node.name).toBe('box');
        // The default box is 1 unit across, so the closest surface point sits at x = 0.5.
        expect(hit.point.x).toBeCloseTo(0.5, 3);
    });

    test('AABB query enumerates overlapping colliders', () => {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const { rapierCollidersInAabb } = require('../../cocos/physics/rapier/extras');

        const a = new Node('inside');
        const b = new Node('outside');
        scene.addChild(a);
        scene.addChild(b);
        a.worldPosition = new Vec3(0, 0, 0);
        b.worldPosition = new Vec3(50, 0, 0);
        a.addComponent(physics.BoxCollider);
        b.addComponent(physics.BoxCollider);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        const found: string[] = [];
        rapierCollidersInAabb(new Vec3(0, 0, 0), new Vec3(2, 2, 2), (c: any): boolean => {
            found.push(c.node.name as string);
            return true;
        });

        expect(found).toContain('inside');
        expect(found).not.toContain('outside');
    });
});
