import { director, game, Game } from "../../cocos/game";
import { physics, PhysicsSystem } from "../../exports/physics-framework";
import "../../exports/physics-rapier";
import { waitForRapierInstantiation } from "../../cocos/physics/rapier/instantiated";
import { Node, Scene } from "../../cocos/scene-graph";
import { geometry, Vec3 } from "../../cocos/core";
import { rapierProjectPoint, rapierForEachContactManifold } from "../../cocos/physics/rapier/extras";

beforeAll(async () => {
    await waitForRapierInstantiation();
    game.emit(Game.EVENT_PRE_SUBSYSTEM_INIT);
    PhysicsSystem.constructAndRegister();
});

describe('rapier scene queries', () => {
    let scene: Scene;
    let box: physics.BoxCollider;
    const ray = new geometry.Ray();

    beforeEach(() => {
        physics.selector.switchTo('rapier');
        scene = new Scene('rapier-queries');
        director.runSceneImmediate(scene);

        // Unit box centred at z = 2, so its near face is at z = 1.5.
        const node = new Node('box');
        scene.addChild(node);
        node.worldPosition = new Vec3(0, 0, 2);
        box = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        box.setGroup(1 << 3);
        director.tick(PhysicsSystem.instance.fixedTimeStep);

        ray.o.set(0, 0, 0);
        ray.d.set(0, 0, 1);
    });

    afterEach(() => {
        scene.destroy();
    });

    test('a collider whose own collision mask is 0 is still hit by group', () => {
        box.setMask(0);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        const sys = PhysicsSystem.instance;

        expect(sys.raycastClosest(ray, 1 << 3)).toBe(true);
        expect(sys.raycastClosestResult.collider.uuid).toBe(box.uuid);
        expect(sys.raycast(ray, 1 << 3)).toBe(true);
        expect(sys.raycastResults.length).toBe(1);
        expect(sys.sweepSphereClosest(ray, 0.1, 1 << 3)).toBe(true);
        expect(sys.sweepSphere(ray, 0.1, 1 << 3)).toBe(true);
    });

    test('the query mask still excludes non-matching groups', () => {
        const sys = PhysicsSystem.instance;
        expect(sys.raycastClosest(ray, 1 << 4)).toBe(false);
        expect(sys.raycast(ray, 1 << 4)).toBe(false);
        expect(sys.sweepSphereClosest(ray, 0.1, 1 << 4)).toBe(false);
    });

    test('groups above bit 15 are not folded onto the low bits', () => {
        box.setGroup(1 << 19);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        const sys = PhysicsSystem.instance;

        expect(sys.raycastClosest(ray, 1 << 3)).toBe(false);
        expect(sys.raycastClosest(ray, 1 << 19)).toBe(true);
    });

    test('a non-unit ray direction still reports world distance', () => {
        ray.d.set(0, 0, 5);
        const sys = PhysicsSystem.instance;
        expect(sys.raycastClosest(ray, -1)).toBe(true);
        expect(sys.raycastClosestResult.distance).toBeCloseTo(1.5, 4);
        expect(sys.raycastClosestResult.hitPoint.z).toBeCloseTo(1.5, 4);
    });

    test('maxDistance bounds the hit', () => {
        const sys = PhysicsSystem.instance;
        expect(sys.raycastClosest(ray, -1, 1.4)).toBe(false);
        expect(sys.raycastClosest(ray, -1, 1.6)).toBe(true);
    });

    test('a zero-length ray hits nothing', () => {
        ray.d.set(0, 0, 0);
        const sys = PhysicsSystem.instance;
        expect(sys.raycastClosest(ray, -1)).toBe(false);
        expect(sys.raycast(ray, -1)).toBe(false);
    });

    test('a ray starting inside a collider reports a hit at distance 0', () => {
        ray.o.set(0, 0, 2);
        const sys = PhysicsSystem.instance;
        expect(sys.raycastClosest(ray, -1)).toBe(true);
        expect(sys.raycastClosestResult.distance).toBeCloseTo(0, 4);
    });

    test('hitNormal follows a rotated collider', () => {
        box.node.setWorldRotationFromEuler(45, 0, 0);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        ray.o.set(0, 0.2, 0);
        const sys = PhysicsSystem.instance;

        expect(sys.raycastClosest(ray, -1)).toBe(true);
        const n = sys.raycastClosestResult.hitNormal;
        expect(n.x).toBeCloseTo(0, 4);
        expect(n.y).toBeCloseTo(Math.SQRT1_2, 3);
        expect(n.z).toBeCloseTo(-Math.SQRT1_2, 3);
    });

    test('a sweep reports the contact point and normal in world space', () => {
        box.node.setWorldRotationFromEuler(0, 45, 0);
        box.node.setWorldPosition(3, 0, 2);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        ray.o.set(3, 0, 0);
        const sys = PhysicsSystem.instance;

        expect(sys.sweepSphereClosest(ray, 0.1, -1)).toBe(true);
        const r = sys.sweepCastClosestResult;
        // Rotated 45 degrees about Y, the near edge of the box is at z = 2 - sqrt(2) / 2.
        const edgeZ = 2 - Math.SQRT1_2;
        expect(r.hitPoint.x).toBeCloseTo(3, 2);
        expect(r.hitPoint.z).toBeCloseTo(edgeZ, 2);
        expect(r.hitNormal.z).toBeLessThan(0);
        expect(Vec3.len(r.hitNormal)).toBeCloseTo(1, 3);
    });

    test('an all-hits sweep stops at 64 hits, closest first', () => {
        for (let i = 0; i < 70; i++) {
            const n = new Node(`extra${i}`);
            scene.addChild(n);
            n.worldPosition = new Vec3(0, 0, 10 + i * 2);
            (n.addComponent(physics.BoxCollider) as physics.BoxCollider).setGroup(1 << 3);
        }
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        const sys = PhysicsSystem.instance;

        expect(sys.sweepSphere(ray, 0.1, -1, 1000)).toBe(true);
        const hits = sys.sweepCastResults;
        expect(hits.length).toBe(64);
        for (let i = 1; i < hits.length; i++) {
            expect(hits[i].distance).toBeGreaterThanOrEqual(hits[i - 1].distance);
        }
    });

    test('rapierProjectPoint finds the closest point on a collider', () => {
        const projection = rapierProjectPoint(new Vec3(0, 0, 0), true);
        expect(projection).not.toBeNull();
        expect(projection!.collider.uuid).toBe(box.uuid);
        expect(projection!.isInside).toBe(false);
        expect(projection!.point.z).toBeCloseTo(1.5, 4);

        const inside = rapierProjectPoint(new Vec3(0, 0, 2), true);
        expect(inside!.isInside).toBe(true);

        expect(rapierProjectPoint(new Vec3(0, 0, 0), true, { predicate: () => false })).toBeNull();
    });

    test('rapierForEachContactManifold walks touching colliders', () => {
        const node = new Node('dynamic');
        scene.addChild(node);
        node.worldPosition = new Vec3(0, 0, 2.7); // overlaps the static box by 0.3
        const other = node.addComponent(physics.BoxCollider) as physics.BoxCollider;
        const rb = node.addComponent(physics.RigidBody) as physics.RigidBody;
        rb.type = physics.RigidBody.Type.DYNAMIC;
        rb.useGravity = false;
        // The fixture box sits in a group the collision matrix leaves empty, so make the pair collide explicitly.
        box.setMask(-1);
        other.setMask(-1);
        for (let i = 0; i < 5; i++) director.tick(PhysicsSystem.instance.fixedTimeStep);

        let manifolds = 0;
        rapierForEachContactManifold(box, other, () => { manifolds++; });
        expect(manifolds).toBeGreaterThan(0);

        // Colliders far apart have no manifold.
        node.worldPosition = new Vec3(0, 0, 50);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        director.tick(PhysicsSystem.instance.fixedTimeStep);
        manifolds = 0;
        rapierForEachContactManifold(box, other, () => { manifolds++; });
        expect(manifolds).toBe(0);
    });

    test('queries after the world is destroyed return false instead of throwing', () => {
        const world = PhysicsSystem.instance.physicsWorld;
        world.destroy();
        const result = PhysicsSystem.instance.raycastClosestResult;
        expect(world.raycastClosest(ray, { mask: -1, group: -1, queryTrigger: true, maxDistance: 10 } as any, result)).toBe(false);
    });
});
