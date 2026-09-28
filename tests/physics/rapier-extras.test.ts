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
});
