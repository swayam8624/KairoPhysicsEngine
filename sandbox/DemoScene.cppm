module;

#include <array>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

export module Kairo.Foundation.PhysicsSandbox.DemoScene;

import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsSandbox.Types;

export namespace kairo::foundation::physics::sandbox
{
    using namespace kairo::foundation::math;

    namespace detail
    {
        [[nodiscard]]
        RigidBodyDesc DynamicSphereBody(
            const Vec3f& position,
            float radius,
            float density = 1.0f)
        {
            RigidBodyDesc desc;
            desc.Type = BodyType::Dynamic;
            desc.State.Position = position;
            desc.Mass = SphereMassProperties(radius, density);
            desc.LinearDamping = 0.02f;
            desc.AngularDamping = 0.02f;
            desc.AllowSleeping = true;
            return desc;
        }

        [[nodiscard]]
        RigidBodyDesc DynamicBoxBody(
            const Vec3f& position,
            const Vec3f& halfExtents,
            float density = 1.0f)
        {
            RigidBodyDesc desc;
            desc.Type = BodyType::Dynamic;
            desc.State.Position = position;
            desc.Mass = BoxMassProperties(halfExtents, density);
            desc.LinearDamping = 0.04f;
            desc.AngularDamping = 0.08f;
            desc.AllowSleeping = true;
            return desc;
        }

        [[nodiscard]]
        RigidBodyDesc StaticBody(
            const Vec3f& position = Vec3f::Zero(),
            const Quaternionf& rotation = Quaternionf::Identity())
        {
            RigidBodyDesc desc;
            desc.Type = BodyType::Static;
            desc.State.Position = position;
            desc.State.Rotation = rotation;
            desc.Mass = StaticMassProperties();
            return desc;
        }

        void ConfigureWorld(
            PhysicsWorld& world)
        {
            world.Settings.VelocityIterations = 18;
            world.Settings.PositionIterations = 8;
            world.Settings.EnableSleeping = true;
            world.Settings.SleepTime = 0.45f;
            world.SetCollisionLayerResponse(
                CollisionLayer::DynamicWorld,
                CollisionLayer::StaticWorld,
                CollisionResponse::Block);
            world.SetCollisionLayerResponse(
                CollisionLayer::DynamicWorld,
                CollisionLayer::DynamicWorld,
                CollisionResponse::Block);
            world.SetCollisionLayerResponse(
                CollisionLayer::DynamicWorld,
                CollisionLayer::Trigger,
                CollisionResponse::Trigger);
        }

        ColliderID AddFloorPlane(
            SandboxScene& scene)
        {
            const BodyID floor =
                scene.World.CreateRigidBody(StaticBody());

            const ColliderID collider =
                scene.World.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

            scene.World.SetCollisionFilter(
                collider,
                CollisionLayer::StaticWorld,
                CollisionLayer::All);

            return collider;
        }

        TrackedBody AddSphere(
            SandboxScene& scene,
            std::string name,
            const Vec3f& position,
            float radius,
            char glyph,
            const PhysicsMaterial& material = {})
        {
            const BodyID body =
                scene.World.CreateRigidBody(DynamicSphereBody(position, radius));

            const ColliderID collider =
                scene.World.AddCollider(body, SphereCollider{ radius }, material);

            scene.World.SetCollisionFilter(
                collider,
                CollisionLayer::DynamicWorld,
                CollisionLayer::All);

            TrackedBody tracked{ body, collider, std::move(name), glyph };
            scene.TrackedBodies.push_back(tracked);
            return tracked;
        }

        TrackedBody AddBox(
            SandboxScene& scene,
            std::string name,
            const Vec3f& position,
            const Vec3f& halfExtents,
            char glyph,
            const PhysicsMaterial& material = {},
            bool useAabbCollider = false)
        {
            const BodyID body =
                scene.World.CreateRigidBody(DynamicBoxBody(position, halfExtents));

            const ColliderID collider =
                useAabbCollider
                    ? scene.World.AddCollider(body, AABBCollider{ halfExtents }, material)
                    : scene.World.AddCollider(body, BoxCollider{ halfExtents }, material);

            scene.World.SetCollisionFilter(
                collider,
                CollisionLayer::DynamicWorld,
                CollisionLayer::All);

            TrackedBody tracked{ body, collider, std::move(name), glyph };
            scene.TrackedBodies.push_back(tracked);
            return tracked;
        }

        void AddStaticRamp(
            SandboxScene& scene,
            const Vec3f& position,
            const Vec3f& halfExtents,
            float radians)
        {
            const BodyID body =
                scene.World.CreateRigidBody(StaticBody(position, RotationAroundZ(radians)));

            const ColliderID collider =
                scene.World.AddCollider(body, BoxCollider{ halfExtents });

            scene.World.SetCollisionFilter(
                collider,
                CollisionLayer::StaticWorld,
                CollisionLayer::All);
        }

        void BuildFallingSphere(
            SandboxScene& scene)
        {
            AddFloorPlane(scene);
            AddSphere(scene, "falling_sphere", Vec3f{ 0.0f, 4.0f, 0.0f }, 0.5f, 'o');
        }

        void BuildSphereCollision(
            SandboxScene& scene)
        {
            scene.World.Gravity = Vec3f::Zero();
            scene.World.Settings.EnableSleeping = false;

            const TrackedBody left =
                AddSphere(scene, "left_sphere", Vec3f{ -2.0f, 1.0f, 0.0f }, 0.5f, 'o');

            const TrackedBody right =
                AddSphere(scene, "right_sphere", Vec3f{ 2.0f, 1.0f, 0.0f }, 0.5f, 'O');

            scene.World.Bodies().at(left.Body).EnableGravity = false;
            scene.World.Bodies().at(right.Body).EnableGravity = false;
            scene.World.Bodies().at(left.Body).State.LinearVelocity = Vec3f{ 2.8f, 0.0f, 0.0f };
            scene.World.Bodies().at(right.Body).State.LinearVelocity = Vec3f{ -2.8f, 0.0f, 0.0f };
        }

        void BuildBoxStack(
            SandboxScene& scene)
        {
            AddFloorPlane(scene);

            for (int row = 0; row < 4; ++row)
            {
                for (int column = 0; column < 4 - row; ++column)
                {
                    const float x =
                        -1.15f + static_cast<float>(column) * 0.72f + static_cast<float>(row) * 0.36f;

                    AddBox(
                        scene,
                        "aabb_box_" + std::to_string(row) + "_" + std::to_string(column),
                        Vec3f{ x, 0.45f + static_cast<float>(row) * 0.72f, 0.0f },
                        Vec3f{ 0.32f, 0.32f, 0.32f },
                        '#',
                        {},
                        true);
                }
            }
        }

        void BuildFrictionRamp(
            SandboxScene& scene)
        {
            AddFloorPlane(scene);
            AddStaticRamp(scene, Vec3f{ 0.0f, 1.0f, 0.0f }, Vec3f{ 3.6f, 0.14f, 0.5f }, -0.28f);

            PhysicsMaterial highFriction;
            highFriction.StaticFriction = 1.0f;
            highFriction.DynamicFriction = 0.75f;
            highFriction.Restitution = 0.0f;

            AddBox(
                scene,
                "friction_box",
                Vec3f{ -2.6f, 2.1f, 0.0f },
                Vec3f{ 0.35f, 0.35f, 0.35f },
                '#',
                highFriction);
        }

        void BuildRestitutionTest(
            SandboxScene& scene)
        {
            AddFloorPlane(scene);

            std::array<float, 4> restitution{ 0.0f, 0.35f, 0.65f, 0.95f };
            for (std::size_t i = 0; i < restitution.size(); ++i)
            {
                PhysicsMaterial material;
                material.Restitution = restitution.at(i);
                material.StaticFriction = 0.2f;
                material.DynamicFriction = 0.15f;

                AddSphere(
                    scene,
                    "bounce_" + std::to_string(i),
                    Vec3f{ -2.25f + static_cast<float>(i) * 1.5f, 4.5f, 0.0f },
                    0.35f,
                    static_cast<char>('0' + static_cast<int>(i)),
                    material);
            }
        }

        void BuildSleepingTest(
            SandboxScene& scene)
        {
            AddFloorPlane(scene);
            scene.World.Settings.SleepTime = 0.25f;
            scene.World.Settings.SleepLinearSpeed = 0.08f;
            scene.World.Settings.SleepAngularSpeed = 0.08f;

            for (int i = 0; i < 8; ++i)
            {
                AddBox(
                    scene,
                    "sleep_box_" + std::to_string(i),
                    Vec3f{ 0.0f, 0.45f + static_cast<float>(i) * 0.68f, 0.0f },
                    Vec3f{ 0.30f, 0.30f, 0.30f },
                    '#',
                    {},
                    true);
            }
        }

        void BuildSphereStress(
            SandboxScene& scene,
            int count)
        {
            AddFloorPlane(scene);

            const int columns =
                count <= 100 ? 10 : 25;

            for (int i = 0; i < count; ++i)
            {
                const int xIndex =
                    i % columns;

                const int yIndex =
                    i / columns;

                AddSphere(
                    scene,
                    "stress_sphere_" + std::to_string(i),
                    Vec3f
                    {
                        -4.5f + static_cast<float>(xIndex) * 0.38f,
                        2.0f + static_cast<float>(yIndex) * 0.38f,
                        0.0f
                    },
                    0.16f,
                    'o');
            }
        }

        void BuildProjectileWall(SandboxScene& scene, bool trigger)
        {
            scene.World.Gravity = Vec3f::Zero();
            const BodyID target = scene.World.CreateRigidBody(StaticBody(Vec3f{ 3.0f, 1.0f, 0.0f }));
            const ColliderID collider = scene.World.AddCollider(
                target, trigger ? ColliderShape{ SphereCollider{ 0.6f } } : ColliderShape{ AABBCollider{ Vec3f{ 0.08f, 1.0f, 1.0f } } });
            scene.World.SetCollisionFilter(collider, trigger ? CollisionLayer::Trigger : CollisionLayer::StaticWorld, CollisionLayer::All);
            scene.World.SetColliderTrigger(collider, trigger);

            ProjectileDesc projectile;
            projectile.Mode = ProjectileMode::Ballistic;
            projectile.Position = Vec3f{ -3.0f, 1.0f, 0.0f };
            projectile.Velocity = Vec3f{ 40.0f, 0.0f, 0.0f };
            projectile.Radius = 0.08f;
            projectile.Response = trigger ? ProjectileImpactResponse::Pierce : ProjectileImpactResponse::Destroy;
            projectile.Lifetime = 1.0f;
            projectile.MaxDistance = 20.0f;
            [[maybe_unused]] const ProjectileID projectileID =
                scene.Projectiles.Spawn(projectile);
        }

        void BuildWaterBuoyancy(SandboxScene& scene)
        {
            scene.World.Gravity = Vec3f{ 0.0f, -9.81f, 0.0f };
            const TrackedBody floatBody = AddSphere(scene, "floating_sphere", Vec3f{ 0.0f, 0.25f, 0.0f }, 0.5f, 'o');
            [[maybe_unused]] const WaterVolumeID waterVolume = scene.Water.AddWaterVolume({
                AABBf::FromMinMax(Vec3f{ -4.0f, -1.0f, -2.0f }, Vec3f{ 4.0f, 1.0f, 2.0f }),
                1.0f, 1.25f, 0.3f, CollisionLayer::Fluid
            });
            scene.Water.RegisterBody(floatBody.Body, { 1.0f, 1.0f, 1.0f, 1.0f });
        }
    }

    using namespace detail;

    /// Input: scenario enum.
    /// Output: stable command-line scene name.
    /// Task: keep docs, scenes, and CLI output spelling consistent.
    [[nodiscard]]
    std::string ScenarioName(
        SandboxScenario scenario)
    {
        switch (scenario)
        {
        case SandboxScenario::FallingSphere:
            return "falling-sphere";
        case SandboxScenario::SphereCollision:
            return "sphere-collision";
        case SandboxScenario::BoxStack:
            return "box-stack";
        case SandboxScenario::FrictionRamp:
            return "friction-ramp";
        case SandboxScenario::RestitutionTest:
            return "restitution-test";
        case SandboxScenario::SleepingTest:
            return "sleeping-test";
        case SandboxScenario::Stress100Spheres:
            return "stress-100";
        case SandboxScenario::Stress500Spheres:
            return "stress-500";
        case SandboxScenario::ProjectileWall:
            return "projectile-wall";
        case SandboxScenario::TriggerVolume:
            return "trigger-volume";
        case SandboxScenario::WaterBuoyancy:
            return "water-buoyancy";
        }

        return "unknown";
    }

    /// Input: command-line scene name.
    /// Output: parsed scenario enum, or throws for unknown names.
    /// Task: fail early with a useful message instead of silently choosing the
    /// wrong physical setup.
    [[nodiscard]]
    SandboxScenario ParseScenarioName(
        std::string_view name)
    {
        if (name == "falling-sphere")
        {
            return SandboxScenario::FallingSphere;
        }
        if (name == "sphere-collision")
        {
            return SandboxScenario::SphereCollision;
        }
        if (name == "box-stack")
        {
            return SandboxScenario::BoxStack;
        }
        if (name == "friction-ramp")
        {
            return SandboxScenario::FrictionRamp;
        }
        if (name == "restitution-test")
        {
            return SandboxScenario::RestitutionTest;
        }
        if (name == "sleeping-test")
        {
            return SandboxScenario::SleepingTest;
        }
        if (name == "stress-100")
        {
            return SandboxScenario::Stress100Spheres;
        }
        if (name == "stress-500")
        {
            return SandboxScenario::Stress500Spheres;
        }
        if (name == "projectile-wall") { return SandboxScenario::ProjectileWall; }
        if (name == "trigger-volume") { return SandboxScenario::TriggerVolume; }
        if (name == "water-buoyancy") { return SandboxScenario::WaterBuoyancy; }

        throw std::invalid_argument(
            "Unknown sandbox scenario '" + std::string(name) + "'.");
    }

    /// Input: scenario enum.
    /// Output: fully constructed physics world plus tracked render metadata.
    /// Task: build reusable demo scenes for terminal output, GLFW rendering,
    /// benchmark CSV, and later ray-traced frame export.
    [[nodiscard]]
    SandboxScene MakeDemoScene(
        SandboxScenario scenario)
    {
        SandboxScene scene;
        ConfigureWorld(scene.World);
        scene.Name = ScenarioName(scenario);

        switch (scenario)
        {
        case SandboxScenario::FallingSphere:
            scene.Description = "single dynamic sphere falling onto an infinite plane";
            BuildFallingSphere(scene);
            break;
        case SandboxScenario::SphereCollision:
            scene.Description = "two dynamic spheres colliding head-on without gravity";
            BuildSphereCollision(scene);
            break;
        case SandboxScenario::BoxStack:
            scene.Description = "axis-aligned box pyramid settling onto the floor";
            BuildBoxStack(scene);
            break;
        case SandboxScenario::FrictionRamp:
            scene.Description = "box sliding down an inclined static ramp with friction";
            BuildFrictionRamp(scene);
            break;
        case SandboxScenario::RestitutionTest:
            scene.Description = "multiple balls with different restitution values";
            BuildRestitutionTest(scene);
            break;
        case SandboxScenario::SleepingTest:
            scene.Description = "vertical box stack used to inspect sleep transitions";
            BuildSleepingTest(scene);
            break;
        case SandboxScenario::Stress100Spheres:
            scene.Description = "100 sphere broadphase and solver stress setup";
            BuildSphereStress(scene, 100);
            break;
        case SandboxScenario::Stress500Spheres:
            scene.Description = "500 sphere broadphase stress setup";
            BuildSphereStress(scene, 500);
            break;
        case SandboxScenario::ProjectileWall:
            scene.Description = "continuous swept-sphere projectile against a thin static wall";
            BuildProjectileWall(scene, false);
            break;
        case SandboxScenario::TriggerVolume:
            scene.Description = "piercing projectile generating a trigger-volume hit";
            BuildProjectileWall(scene, true);
            break;
        case SandboxScenario::WaterBuoyancy:
            scene.Description = "submerged sphere with buoyancy and drag forces";
            BuildWaterBuoyancy(scene);
            break;
        }

        return scene;
    }

    /// Input: none.
    /// Output: human-readable list of supported scenario names.
    /// Task: keep CLI usage and README examples synchronized with code.
    [[nodiscard]]
    std::string SupportedScenarioList()
    {
        return
            "falling-sphere, sphere-collision, box-stack, friction-ramp, "
            "restitution-test, sleeping-test, stress-100, stress-500, "
            "projectile-wall, trigger-volume, water-buoyancy";
    }
}
