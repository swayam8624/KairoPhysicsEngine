#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

namespace
{
    std::vector<Vec3f> RunDeterministicStressScene()
    {
        PhysicsWorld world;
        world.Settings.EnableSleeping = false;

        RigidBodyDesc floorDesc;
        floorDesc.Type = BodyType::Static;
        floorDesc.Mass = StaticMassProperties();
        const BodyID floor = world.CreateRigidBody(floorDesc);
        (void)world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

        std::vector<BodyID> dynamicBodies;
        for (int z = 0; z < 6; ++z)
            for (int x = 0; x < 6; ++x)
            {
                RigidBodyDesc body;
                body.Type = BodyType::Dynamic;
                body.State.Position = Vec3f{
                    static_cast<float>(x) * 1.05f - 2.625f,
                    1.0f + static_cast<float>((x + z) % 4) * 1.05f,
                    static_cast<float>(z) * 1.05f - 2.625f };
                body.Mass = SphereMassProperties(0.45f, 1.0f);
                const BodyID id = world.CreateRigidBody(body);
                (void)world.AddCollider(id, SphereCollider{ 0.45f });
                dynamicBodies.push_back(id);
            }

        for (int step = 0; step < 360; ++step)
        {
            world.Step(1.0f / 120.0f);
            const auto& profile = world.LastStepProfile();
            REQUIRE(std::isfinite(profile.StepMs));
            REQUIRE(std::isfinite(profile.BroadphaseMs));
            REQUIRE(std::isfinite(profile.NarrowphaseMs));
            REQUIRE(std::isfinite(profile.SolverMs));
        }

        std::vector<Vec3f> result;
        result.reserve(dynamicBodies.size());
        for (const BodyID id : dynamicBodies)
        {
            const Vec3f position = world.Bodies().at(id).State.Position;
            REQUIRE(std::isfinite(position.x));
            REQUIRE(std::isfinite(position.y));
            REQUIRE(std::isfinite(position.z));
            result.push_back(position);
        }
        return result;
    }
}

TEST_CASE("physics stress scene is stable and deterministic across repeated runs",
    "[PhysicsEngine][Scale][Determinism]")
{
    const auto first = RunDeterministicStressScene();
    const auto second = RunDeterministicStressScene();
    REQUIRE(first.size() == second.size());
    for (std::size_t index = 0u; index < first.size(); ++index)
    {
        CHECK(first[index].x == Catch::Approx(second[index].x).margin(1.0e-6f));
        CHECK(first[index].y == Catch::Approx(second[index].y).margin(1.0e-6f));
        CHECK(first[index].z == Catch::Approx(second[index].z).margin(1.0e-6f));
    }
}
