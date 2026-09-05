from pathlib import Path
import re

# -----------------------------------------------------------------------------
# RigidBody: public per-body discrete/continuous motion quality.
# -----------------------------------------------------------------------------
p = Path('RigidBody.cppm')
s = p.read_text()

if '#include <cstdint>\n' not in s:
    s = s.replace('#include <stdexcept>\n', '#include <cstdint>\n#include <stdexcept>\n', 1)

marker = '    struct RigidBodyDesc final\n'
if marker not in s:
    raise SystemExit('RigidBodyDesc marker not found')
if 'enum class CollisionDetectionMode' not in s:
    s = s.replace(
        marker,
        '''    /// Selects whether final dynamic translation is integrated discretely\n'''
        '''    /// or clipped against a conservative time-of-impact sweep.\n'''
        '''    enum class CollisionDetectionMode : std::uint8_t\n'''
        '''    {\n'''
        '''        Discrete,\n'''
        '''        Continuous\n'''
        '''    };\n\n'''
        + marker,
        1)

old = '''        float MaxAngularSpeed = 1000.0f;\n        bool AllowSleeping = true;\n'''
new = '''        float MaxAngularSpeed = 1000.0f;\n        CollisionDetectionMode CollisionDetection = CollisionDetectionMode::Discrete;\n        bool AllowSleeping = true;\n'''
if s.count(old) < 2:
    raise SystemExit('RigidBody descriptor/record insertion markers not found')
s = s.replace(old, new, 2)

validation_marker = '''        RequirePositive(desc.MaxAngularSpeed, "RigidBodyDesc.MaxAngularSpeed");\n\n'''
if validation_marker not in s:
    raise SystemExit('RigidBody validation marker not found')
validation = '''        RequirePositive(desc.MaxAngularSpeed, "RigidBodyDesc.MaxAngularSpeed");\n\n        switch (desc.CollisionDetection)\n        {\n        case CollisionDetectionMode::Discrete:\n        case CollisionDetectionMode::Continuous:\n            break;\n        default:\n            throw std::invalid_argument(\n                "RigidBodyDesc.CollisionDetection contains an invalid enum value.");\n        }\n\n'''
s = s.replace(validation_marker, validation, 1)

copy_marker = '''        body.MaxAngularSpeed = desc.MaxAngularSpeed;\n        body.AllowSleeping = desc.AllowSleeping;\n'''
if copy_marker not in s:
    raise SystemExit('RigidBody copy marker not found')
s = s.replace(
    copy_marker,
    '''        body.MaxAngularSpeed = desc.MaxAngularSpeed;\n        body.CollisionDetection = desc.CollisionDetection;\n        body.AllowSleeping = desc.AllowSleeping;\n''',
    1)
p.write_text(s)

# -----------------------------------------------------------------------------
# PhysicsWorld: conservative TOI clipping and impact impulse.
# -----------------------------------------------------------------------------
p = Path('PhysicsWorld.cppm')
s = p.read_text()

setter_marker = '''        void AddBodyForce(\n'''
if setter_marker not in s:
    raise SystemExit('AddBodyForce marker not found')
if 'SetBodyCollisionDetectionMode' not in s:
    setter = '''        /// Input: active body id and desired collision-detection mode.\n        /// Output: body integration policy updated for subsequent steps.\n        /// Task: opt fast dynamic bodies into conservative rigid-body CCD while\n        /// preserving discrete integration as the inexpensive default.\n        void SetBodyCollisionDetectionMode(\n            BodyID body,\n            CollisionDetectionMode mode)\n        {\n            RigidBody& record =\n                RequireMutableBody(body, "SetBodyCollisionDetectionMode");\n\n            switch (mode)\n            {\n            case CollisionDetectionMode::Discrete:\n            case CollisionDetectionMode::Continuous:\n                record.CollisionDetection = mode;\n                return;\n            }\n\n            throw std::invalid_argument(\n                "SetBodyCollisionDetectionMode failed: invalid collision detection mode.");\n        }\n\n'''
    s = s.replace(setter_marker, setter + setter_marker, 1)

if 'IntegrateDynamicVelocities(dt);' not in s:
    raise SystemExit('Step integration call not found')
s = s.replace('IntegrateDynamicVelocities(dt);', 'IntegrateDynamicVelocitiesWithCCD(dt);', 1)

pattern = re.compile(
    r'''        void IntegrateDynamicVelocities\(\n            float dt\)\n        \{\n.*?\n        \}\n\n        void ClearForceAccumulators\(\) noexcept''',
    re.S)
match = pattern.search(s)
if not match:
    raise SystemExit('IntegrateDynamicVelocities function block not found')

replacement = r'''        struct ContinuousImpact final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            Vec3f NormalAtoB = Vec3f::UnitX();
            float TimeOfImpact = 1.0f;
        };

        [[nodiscard]]
        static float ColliderSweepRadius(
            const Collider& collider)
        {
            if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
            {
                return sphere->Radius;
            }
            if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
            {
                return capsule->HalfHeight + capsule->Radius;
            }
            if (const auto* box = std::get_if<AABBCollider>(&collider.Shape))
            {
                return box->HalfExtents.Length();
            }
            if (const auto* box = std::get_if<BoxCollider>(&collider.Shape))
            {
                return box->HalfExtents.Length();
            }
            if (const auto* hull = std::get_if<ConvexHullCollider>(&collider.Shape))
            {
                float radius = 0.0f;
                for (const Vec3f& vertex : hull->Vertices)
                {
                    radius = std::max(radius, vertex.Length());
                }
                return radius;
            }

            // Infinite planes cannot be swept as moving source volumes and
            // triangle meshes are intentionally static-only in this engine.
            return 0.0f;
        }

        void ApplyContinuousImpactImpulse(
            const ContinuousImpact& impact)
        {
            if (impact.BodyA >= m_Bodies.size() ||
                impact.BodyB >= m_Bodies.size() ||
                impact.ColliderA >= m_Colliders.size() ||
                impact.ColliderB >= m_Colliders.size())
            {
                return;
            }

            RigidBody& bodyA = m_Bodies.at(impact.BodyA);
            RigidBody& bodyB = m_Bodies.at(impact.BodyB);
            if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
            {
                return;
            }

            const float inverseMassA =
                IsDynamicBodyType(bodyA) ? bodyA.Mass.InverseMass : 0.0f;
            const float inverseMassB =
                IsDynamicBodyType(bodyB) ? bodyB.Mass.InverseMass : 0.0f;
            const float inverseMassSum = inverseMassA + inverseMassB;
            if (inverseMassSum <= 0.0f)
            {
                return;
            }

            const Vec3f normal =
                SafeNormalize(impact.NormalAtoB, Vec3f::UnitX());
            const float relativeNormalVelocity =
                Dot(bodyB.State.LinearVelocity - bodyA.State.LinearVelocity, normal);
            if (relativeNormalVelocity >= 0.0f)
            {
                return;
            }

            const Collider& colliderA = m_Colliders.at(impact.ColliderA);
            const Collider& colliderB = m_Colliders.at(impact.ColliderB);
            const float restitution = MixRestitution(colliderA.Material, colliderB.Material);
            const float impulseMagnitude =
                -(1.0f + restitution) * relativeNormalVelocity / inverseMassSum;
            const Vec3f impulse = normal * impulseMagnitude;

            if (inverseMassA > 0.0f)
            {
                WakeRigidBody(bodyA);
                bodyA.State.LinearVelocity -= impulse * inverseMassA;
            }
            if (inverseMassB > 0.0f)
            {
                WakeRigidBody(bodyB);
                bodyB.State.LinearVelocity += impulse * inverseMassB;
            }
        }

        void IntegrateDynamicVelocitiesWithCCD(
            float dt)
        {
            constexpr float minimumTravelSq = 1.0e-12f;
            constexpr float minimumTOI = 1.0e-5f;
            constexpr float toiSafety = 1.0e-4f;
            constexpr float impactWindow = 2.0e-3f;

            std::vector<float> motionFractions(m_Bodies.size(), 1.0f);
            std::vector<ContinuousImpact> impacts;

            for (const RigidBody& movingBody : m_Bodies)
            {
                if (!IsDynamic(movingBody) ||
                    movingBody.CollisionDetection != CollisionDetectionMode::Continuous)
                {
                    continue;
                }

                const Vec3f movingDisplacement =
                    movingBody.State.LinearVelocity * dt;
                if (movingDisplacement.LengthSquared() <= minimumTravelSq)
                {
                    continue;
                }

                for (const Collider& movingCollider : m_Colliders)
                {
                    if (!IsActiveCollider(movingCollider) ||
                        movingCollider.Body != movingBody.ID ||
                        IsInfiniteCollider(movingCollider))
                    {
                        continue;
                    }

                    const float radius = ColliderSweepRadius(movingCollider);
                    if (radius <= 0.0f)
                    {
                        continue;
                    }

                    const Vec3f start =
                        WorldColliderCenter(movingBody, movingCollider);

                    for (const Collider& targetCollider : m_Colliders)
                    {
                        if (!IsActiveCollider(targetCollider) ||
                            targetCollider.Body == movingBody.ID ||
                            targetCollider.Body >= m_Bodies.size() ||
                            !CollisionFiltersAllow(movingCollider, targetCollider) ||
                            ResolveCollisionResponse(movingCollider, targetCollider) !=
                                CollisionResponse::Block)
                        {
                            continue;
                        }

                        const RigidBody& targetBody =
                            m_Bodies.at(targetCollider.Body);
                        if (!IsActiveBody(targetBody))
                        {
                            continue;
                        }

                        // Dynamic targets are swept in relative translation so
                        // two fast bodies cannot cross between their old centers.
                        // Kinematic bodies have already advanced earlier in Step,
                        // so their current transform is the target pose here.
                        const Vec3f targetDisplacement =
                            IsDynamic(targetBody)
                                ? targetBody.State.LinearVelocity * dt
                                : Vec3f::Zero();
                        const Vec3f relativeDisplacement =
                            movingDisplacement - targetDisplacement;
                        if (relativeDisplacement.LengthSquared() <= minimumTravelSq)
                        {
                            continue;
                        }

                        const auto hit = SweepSphereCollider(
                            targetBody,
                            targetCollider,
                            start,
                            relativeDisplacement,
                            radius);
                        if (!hit ||
                            hit->TimeOfImpact <= minimumTOI ||
                            hit->TimeOfImpact > 1.0f ||
                            Dot(relativeDisplacement, hit->Normal) >= -1.0e-6f)
                        {
                            continue;
                        }

                        const float fraction =
                            std::max(0.0f, hit->TimeOfImpact - toiSafety);
                        motionFractions.at(movingBody.ID) =
                            std::min(motionFractions.at(movingBody.ID), fraction);
                        if (IsDynamicBodyType(targetBody))
                        {
                            motionFractions.at(targetBody.ID) =
                                std::min(motionFractions.at(targetBody.ID), fraction);
                        }

                        impacts.push_back(
                            ContinuousImpact
                            {
                                movingBody.ID,
                                targetBody.ID,
                                movingCollider.ID,
                                targetCollider.ID,
                                -hit->Normal,
                                hit->TimeOfImpact
                            });
                    }
                }
            }

            // Integrate every dynamic body once. A discrete body can still be
            // clipped when it is the other participant in a continuous pair;
            // this preserves the pair's relative TOI rather than allowing the
            // discrete participant to move through the stopped continuous body.
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamic(body))
                {
                    continue;
                }

                const float fraction =
                    body.ID < motionFractions.size()
                        ? motionFractions.at(body.ID)
                        : 1.0f;
                AdvanceMotionState(body.State, dt * fraction);
            }

            std::sort(
                impacts.begin(),
                impacts.end(),
                [](const ContinuousImpact& lhs, const ContinuousImpact& rhs)
                {
                    if (lhs.TimeOfImpact != rhs.TimeOfImpact)
                    {
                        return lhs.TimeOfImpact < rhs.TimeOfImpact;
                    }
                    if (lhs.ColliderA != rhs.ColliderA)
                    {
                        return lhs.ColliderA < rhs.ColliderA;
                    }
                    return lhs.ColliderB < rhs.ColliderB;
                });

            std::vector<BroadphasePair> resolvedPairs;
            for (const ContinuousImpact& impact : impacts)
            {
                const float fractionA = motionFractions.at(impact.BodyA);
                const float fractionB =
                    impact.BodyB < motionFractions.size()
                        ? motionFractions.at(impact.BodyB)
                        : 1.0f;
                if (impact.TimeOfImpact > fractionA + impactWindow ||
                    (IsDynamicBodyType(m_Bodies.at(impact.BodyB)) &&
                     impact.TimeOfImpact > fractionB + impactWindow))
                {
                    continue;
                }

                const BroadphasePair pair =
                    OrderedBroadphasePair(impact.ColliderA, impact.ColliderB);
                if (std::find(resolvedPairs.begin(), resolvedPairs.end(), pair) !=
                    resolvedPairs.end())
                {
                    continue;
                }

                resolvedPairs.push_back(pair);
                ApplyContinuousImpactImpulse(impact);
            }
        }

        void ClearForceAccumulators() noexcept'''

s = s[:match.start()] + replacement + s[match.end():]
p.write_text(s)

# -----------------------------------------------------------------------------
# Regression tests.
# -----------------------------------------------------------------------------
p = Path('tests/PhysicsEngineTests.cpp')
s = p.read_text()
if 'Continuous sphere rigid body does not tunnel through a plane' not in s:
    insertion = r'''
TEST_CASE("Continuous sphere rigid body does not tunnel through a plane",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    const BodyID floorBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID floorCollider =
        world.AddCollider(floorBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    RigidBodyDesc fastDesc =
        DynamicSphereBody(Vec3f{ 0.0f, 2.0f, 0.0f }, 0.25f);
    fastDesc.State.LinearVelocity = Vec3f{ 0.0f, -200.0f, 0.0f };
    fastDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID fastBody = world.CreateRigidBody(fastDesc);
    [[maybe_unused]] const ColliderID fastCollider =
        world.AddCollider(fastBody, SphereCollider{ 0.25f });

    world.Step(1.0f / 60.0f);

    CHECK(world.Bodies().at(fastBody).State.Position.y >= 0.24f);
    CHECK(world.Bodies().at(fastBody).State.LinearVelocity.y >= 0.0f);
}

TEST_CASE("Continuous boxes use conservative shape bounds against thin walls",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    const BodyID wallBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID wallCollider =
        world.AddCollider(
            wallBody,
            AABBCollider{ Vec3f{ 0.05f, 2.0f, 2.0f } });

    const Vec3f halfExtents{ 0.2f, 0.2f, 0.2f };
    RigidBodyDesc fastDesc =
        DynamicBoxBody(Vec3f{ -2.0f, 0.0f, 0.0f }, halfExtents);
    fastDesc.State.LinearVelocity = Vec3f{ 200.0f, 0.0f, 0.0f };
    fastDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID fastBody = world.CreateRigidBody(fastDesc);
    [[maybe_unused]] const ColliderID fastCollider =
        world.AddCollider(fastBody, BoxCollider{ halfExtents });

    world.Step(0.02f);

    CHECK(world.Bodies().at(fastBody).State.Position.x < -0.05f);
    CHECK(world.Bodies().at(fastBody).State.LinearVelocity.x <= 0.0f);
}

TEST_CASE("Continuous rigid body CCD uses relative motion for dynamic targets",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc aDesc =
        DynamicSphereBody(Vec3f{ -1.0f, 0.0f, 0.0f }, 0.2f);
    aDesc.State.LinearVelocity = Vec3f{ 100.0f, 0.0f, 0.0f };
    aDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID a = world.CreateRigidBody(aDesc);
    [[maybe_unused]] const ColliderID colliderA =
        world.AddCollider(a, SphereCollider{ 0.2f });

    RigidBodyDesc bDesc =
        DynamicSphereBody(Vec3f{ 1.0f, 0.0f, 0.0f }, 0.2f);
    bDesc.State.LinearVelocity = Vec3f{ -100.0f, 0.0f, 0.0f };
    const BodyID b = world.CreateRigidBody(bDesc);
    [[maybe_unused]] const ColliderID colliderB =
        world.AddCollider(b, SphereCollider{ 0.2f });

    world.Step(0.012f);

    const float separation =
        world.Bodies().at(b).State.Position.x -
        world.Bodies().at(a).State.Position.x;
    CHECK(separation >= 0.39f);
    CHECK(world.Bodies().at(a).State.Position.x <
        world.Bodies().at(b).State.Position.x);
    CHECK(world.Bodies().at(a).State.LinearVelocity.x <= 0.0f);
    CHECK(world.Bodies().at(b).State.LinearVelocity.x >= 0.0f);
}
'''
    first_test = s.find('\nTEST_CASE(')
    if first_test < 0:
        raise SystemExit('first TEST_CASE marker not found')
    s = s[:first_test] + '\n' + insertion + s[first_test:]
p.write_text(s)

# -----------------------------------------------------------------------------
# Documentation/roadmap.
# -----------------------------------------------------------------------------
p = Path('README.md')
s = p.read_text()

surface_marker = 'Per-body gravity scale, damping, max velocity clamps, sleeping, and wake hooks\n'
if surface_marker not in s:
    raise SystemExit('README surface marker not found')
s = s.replace(
    surface_marker,
    surface_marker +
    'Per-body Discrete/Continuous collision detection with relative-motion TOI clipping\n',
    1)

s = s.replace(
    'Continuous dynamic rigid-body CCD beyond query/projectile sweeps\n',
    '',
    1)
s = s.replace(
    'Near-term rigid-body work should add general CCD, joints, and island solving before larger\nphysics families are added.',
    'Near-term rigid-body work should add joints and island solving before larger\nphysics families are added.',
    1)

query_marker = '`SweepSphere` is the continuous-query bridge for fast gameplay objects. It\n'
if query_marker not in s:
    raise SystemExit('README sweep marker not found')
ccd_docs = '''Rigid bodies default to inexpensive discrete integration. Fast dynamic bodies can\nopt into conservative CCD; the engine sweeps an enclosing sphere for each finite\nconvex collider, evaluates relative dynamic-body translation, clips both bodies to\nthe earliest blocking TOI, and applies a restitution-aware normal impact impulse:\n\n```cpp\nRigidBodyDesc bullet;\nbullet.Type = BodyType::Dynamic;\nbullet.CollisionDetection = CollisionDetectionMode::Continuous;\n// configure mass/state, create body, then attach any finite convex collider\n```\n\nThis is deliberately conservative for boxes/capsules/hulls: their enclosing sweep\nsphere can stop slightly early, but it cannot miss a translation-only tunnel that\nthe bound covers. Angular swept-volume CCD remains a future refinement.\n\n'''
s = s.replace(query_marker, ccd_docs + query_marker, 1)
p.write_text(s)
