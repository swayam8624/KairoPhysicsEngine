module;

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.SolverIsland;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.ContactSolver;
import Kairo.Foundation.PhysicsEngine.Joint;

export namespace kairo::foundation::physics
{
    struct SolverIsland final
    {
        std::vector<BodyID> DynamicBodies;
        std::vector<std::size_t> ContactIndices;
        std::vector<std::size_t> JointIndices;
    };

    namespace island_detail
    {
        class DisjointSet final
        {
        public:
            explicit DisjointSet(std::size_t count)
                : m_Parent(count)
                , m_Rank(count, 0u)
            {
                for (std::size_t i = 0; i < count; ++i)
                {
                    m_Parent[i] = i;
                }
            }

            [[nodiscard]]
            std::size_t Find(std::size_t value)
            {
                if (m_Parent[value] != value)
                {
                    m_Parent[value] = Find(m_Parent[value]);
                }
                return m_Parent[value];
            }

            void Unite(std::size_t a, std::size_t b)
            {
                a = Find(a);
                b = Find(b);
                if (a == b)
                {
                    return;
                }
                if (m_Rank[a] < m_Rank[b])
                {
                    std::swap(a, b);
                }
                m_Parent[b] = a;
                if (m_Rank[a] == m_Rank[b])
                {
                    ++m_Rank[a];
                }
            }

        private:
            std::vector<std::size_t> m_Parent;
            std::vector<std::uint8_t> m_Rank;
        };

        inline void CorrectContactOnce(
            std::vector<RigidBody>& bodies,
            const ContactManifold& manifold,
            const PhysicsStepSettings& settings)
        {
            if (manifold.IsTrigger ||
                manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
            {
                return;
            }
            RigidBody& bodyA = bodies.at(manifold.BodyA);
            RigidBody& bodyB = bodies.at(manifold.BodyB);
            if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
            {
                return;
            }

            const float invMassA = IsDynamic(bodyA) ? bodyA.Mass.InverseMass : 0.0f;
            const float invMassB = IsDynamic(bodyB) ? bodyB.Mass.InverseMass : 0.0f;
            const float inverseMassSum = invMassA + invMassB;
            if (inverseMassSum <= 0.0f)
            {
                return;
            }

            for (const ContactPoint& point : manifold.Points)
            {
                const float totalCorrectionMagnitude = std::min(
                    std::max(point.PenetrationDepth - settings.Slop, 0.0f) /
                        inverseMassSum,
                    settings.MaxPositionCorrection);
                const float correctionMagnitude =
                    totalCorrectionMagnitude /
                    static_cast<float>(settings.PositionIterations);
                const Vec3f correction =
                    SafeNormalize(point.Normal, Vec3f::Up()) * correctionMagnitude;
                if (invMassA > 0.0f)
                {
                    bodyA.State.Position -= correction * invMassA;
                }
                if (invMassB > 0.0f)
                {
                    bodyB.State.Position += correction * invMassB;
                }
            }
        }

        inline void SolveIslandVelocity(
            std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders,
            std::vector<ContactManifold>& contacts,
            std::vector<Joint>& joints,
            const SolverIsland& island,
            const PhysicsStepSettings& settings,
            float dt)
        {
            for (std::uint32_t iteration = 0;
                iteration < settings.VelocityIterations; ++iteration)
            {
                for (const std::size_t contactIndex : island.ContactIndices)
                {
                    SolveContactManifold(
                        bodies, colliders, contacts.at(contactIndex), settings, dt);
                }
                for (const std::size_t jointIndex : island.JointIndices)
                {
                    SolveJointVelocity(
                        bodies, joints.at(jointIndex), settings, dt);
                }
            }
        }

        inline void CorrectIslandPositions(
            std::vector<RigidBody>& bodies,
            const std::vector<ContactManifold>& contacts,
            const std::vector<Joint>& joints,
            const SolverIsland& island,
            const PhysicsStepSettings& settings)
        {
            for (std::uint32_t iteration = 0;
                iteration < settings.PositionIterations; ++iteration)
            {
                for (const std::size_t contactIndex : island.ContactIndices)
                {
                    CorrectContactOnce(bodies, contacts.at(contactIndex), settings);
                }
                for (const std::size_t jointIndex : island.JointIndices)
                {
                    CorrectJointPosition(bodies, joints.at(jointIndex), settings);
                }
            }
        }

        template<typename Function>
        inline void DispatchIslands(
            const std::vector<SolverIsland>& islands,
            bool parallel,
            std::uint32_t minimumParallelCount,
            Function function)
        {
            if (!parallel ||
                islands.size() < static_cast<std::size_t>(minimumParallelCount) ||
                islands.size() < 2u)
            {
                for (std::size_t i = 0; i < islands.size(); ++i)
                {
                    function(i);
                }
                return;
            }

            const unsigned reported = std::thread::hardware_concurrency();
            const std::size_t workerCount = std::min<std::size_t>(
                islands.size(), reported == 0u ? 2u : reported);
            if (workerCount <= 1u)
            {
                for (std::size_t i = 0; i < islands.size(); ++i)
                {
                    function(i);
                }
                return;
            }

            std::atomic<std::size_t> next{ 0u };
            std::vector<std::jthread> workers;
            workers.reserve(workerCount);
            for (std::size_t worker = 0; worker < workerCount; ++worker)
            {
                workers.emplace_back(
                    [&]()
                    {
                        while (true)
                        {
                            const std::size_t index = next.fetch_add(1u);
                            if (index >= islands.size())
                            {
                                return;
                            }
                            function(index);
                        }
                    });
            }
        }
    }

    [[nodiscard]]
    inline std::vector<SolverIsland> BuildSolverIslands(
        const std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const std::vector<Joint>& joints)
    {
        using namespace island_detail;
        DisjointSet sets(bodies.size());

        for (const ContactManifold& contact : contacts)
        {
            if (contact.IsTrigger ||
                contact.BodyA >= bodies.size() || contact.BodyB >= bodies.size())
            {
                continue;
            }
            if (IsDynamicBodyType(bodies.at(contact.BodyA)) &&
                IsDynamicBodyType(bodies.at(contact.BodyB)))
            {
                sets.Unite(contact.BodyA, contact.BodyB);
            }
        }

        for (const Joint& joint : joints)
        {
            if (!IsActiveJoint(joint) ||
                joint.BodyA >= bodies.size() || joint.BodyB >= bodies.size())
            {
                continue;
            }
            if (IsDynamicBodyType(bodies.at(joint.BodyA)) &&
                IsDynamicBodyType(bodies.at(joint.BodyB)))
            {
                sets.Unite(joint.BodyA, joint.BodyB);
            }
        }

        const std::size_t none = bodies.size();
        std::vector<std::size_t> rootToIsland(bodies.size(), none);
        std::vector<SolverIsland> islands;

        for (const RigidBody& body : bodies)
        {
            if (!IsDynamicBodyType(body))
            {
                continue;
            }
            const std::size_t root = sets.Find(body.ID);
            if (rootToIsland[root] == none)
            {
                rootToIsland[root] = islands.size();
                islands.push_back({});
            }
            islands[rootToIsland[root]].DynamicBodies.push_back(body.ID);
        }

        const auto islandForBodies =
            [&](BodyID bodyA, BodyID bodyB) -> std::size_t
            {
                if (bodyA < bodies.size() && IsDynamicBodyType(bodies.at(bodyA)))
                {
                    return rootToIsland[sets.Find(bodyA)];
                }
                if (bodyB < bodies.size() && IsDynamicBodyType(bodies.at(bodyB)))
                {
                    return rootToIsland[sets.Find(bodyB)];
                }
                return none;
            };

        for (std::size_t index = 0; index < contacts.size(); ++index)
        {
            const ContactManifold& contact = contacts[index];
            if (contact.IsTrigger)
            {
                continue;
            }
            const std::size_t island = islandForBodies(contact.BodyA, contact.BodyB);
            if (island != none)
            {
                islands[island].ContactIndices.push_back(index);
            }
        }

        for (std::size_t index = 0; index < joints.size(); ++index)
        {
            const Joint& joint = joints[index];
            if (!IsActiveJoint(joint))
            {
                continue;
            }
            const std::size_t island = islandForBodies(joint.BodyA, joint.BodyB);
            if (island != none)
            {
                islands[island].JointIndices.push_back(index);
            }
        }

        return islands;
    }

    inline void SolveSolverIslands(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        std::vector<ContactManifold>& contacts,
        std::vector<Joint>& joints,
        const std::vector<SolverIsland>& islands,
        const PhysicsStepSettings& settings,
        float dt)
    {
        island_detail::DispatchIslands(
            islands,
            settings.EnableParallelIslands,
            settings.ParallelIslandMinCount,
            [&](std::size_t index)
            {
                island_detail::SolveIslandVelocity(
                    bodies, colliders, contacts, joints,
                    islands[index], settings, dt);
            });
    }

    inline void CorrectSolverIslandPositions(
        std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const std::vector<Joint>& joints,
        const std::vector<SolverIsland>& islands,
        const PhysicsStepSettings& settings)
    {
        island_detail::DispatchIslands(
            islands,
            settings.EnableParallelIslands,
            settings.ParallelIslandMinCount,
            [&](std::size_t index)
            {
                island_detail::CorrectIslandPositions(
                    bodies, contacts, joints,
                    islands[index], settings);
            });
    }
}
