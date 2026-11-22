// TEACHING MAP: This module partitions PhysicsWorld's contacts and joints
// into disjoint groups of *dynamic* bodies before sequential-impulse solving.
// The union-find structure merges dynamic bodies connected by a contact/joint;
// shared static or kinematic anchors do not themselves merge islands.
//
// SolverIsland stores indices into the current contact/joint arrays, not
// independent copies. Do not retain it across topology changes/steps.
// DispatchIslands may use std::jthread workers; safe parallelism depends on
// each island writing only its own dynamic bodies/constraints. A new constraint
// type that mutates shared objects can invalidate that assumption.
//
// Dependency flow: PhysicsWorld -> BuildSolverIslands ->
// SolveIslandVelocity(ContactSolver, Joint) / CorrectIslandPositions ->
// PhysicsWorld's final integration and event notification. This is neither
// broadphase nor collision geometry generation.
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
    // DynamicBodies is the ownership/partition key for safe per-island
    // writes. ContactIndices and JointIndices refer to caller-owned arrays.
    // A short-lived *view* over the current world step. DynamicBodies
    // identifies writable bodies; ContactIndices/JointIndices are
    // offsets into the caller's existing vectors. Never retain these
    // indices across contact recomputation or topology changes.
    struct SolverIsland final
    {
        std::vector<BodyID> DynamicBodies;
        std::vector<std::size_t> ContactIndices;
        std::vector<std::size_t> JointIndices;
    };

    namespace island_detail
    {
        // Union-find with path compression and union by rank.
        // Here parent indices correspond to BodyID slots, not pointers.
        // A collision/joint connects two *dynamic* bodies into an island.
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
            // Find the connected-component root with path compression.
            // Parent slots are BodyID indices; inverse-mass policy is
            // checked before Unite, not inside the disjoint-set itself.
            std::size_t Find(std::size_t value)
            {
                if (m_Parent[value] != value)
                {
                    m_Parent[value] = Find(m_Parent[value]);
                }
                return m_Parent[value];
            }

            // Union by rank keeps nearly constant amortized lookup
            // cost. Dynamic/dynamic contact or joint edges merge groups;
            // static/kinematic anchor edges deliberately do *not*.
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

        // Unlike the velocity pass, this is positional stabilization.
        // Mass-weighted correction reduces interpenetration beyond the slop
        // threshold. It does not replace geometry update or continuous CCD.
        // A separate position-stabilization projection. Divide
        // correction among dynamic bodies proportional to inverse mass,
        // preserving static/kinematic anchors. The correction is based
        // on the manifold's stored penetration depth, not a freshly
        // recomputed contact for every iteration.
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

        // Iterate all contacts and then all joints for this island on
        // each solver pass. This is the inner loop that applies the
        // PhysicsMath impulse/effective-mass operations via collaborating
        // ContactSolver and Joint modules.
        // Gauss-Seidel order *within* an island: for each velocity
        // iteration solve contacts, then joints, so later constraints
        // observe body velocities changed by earlier impulses.
        // Different islands may run independently and in parallel.
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

        // Reuse the same partition for positional contact and joint
        // constraints after velocity solving. Iterative correction
        // and fixed dt integration are separate phases in World::Step.
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
        // Worker threads claim independent island indices through an
        // atomic counter. The vector of jthreads joins at scope exit; each
        // callback must only mutate data owned by that island. Changing
        // shared state from callback code would introduce data races.
        // Parallel only above the configured island count threshold;
        // otherwise avoid thread startup overhead. Worker jthreads
        // atomically claim a distinct island index and join at scope exit.
        // Safety is *structural*: no two islands may mutate the same
        // dynamic body. Static anchors can be shared because no solver
        // update writes to their state.
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
    // Build groups in two passes: first union dynamic/dynamic edges from
    // non-trigger contacts and active joints, then attach each constraint
    // to the island containing its dynamic endpoint. Static anchors can
    // appear in multiple islands because they are not impulse-updated.
    inline std::vector<SolverIsland> BuildSolverIslands(
        const std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const std::vector<Joint>& joints)
    {
        using namespace island_detail;
        // One union-find element per BodyID slot. Inactive/non-dynamic
        // bodies never become DynamicBodies; static anchors are used
        // only as constraint endpoints, not as unions connecting islands.
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

        // Attach each constraint to the island of its dynamic endpoint.
        // If neither endpoint can move, there is no island to solve.
        // Dynamic/dynamic endpoints have already been unioned above.
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

    // Main world-facing velocity solve. See PhysicsWorld::Step for the
    // surrounding warm start, position correction, CCD and sleep phases.
    // Called by PhysicsWorld *after* restoring cached contact impulses.
    // Per-island contacts and joints share writable body state only
    // within that island. Settings choose sequential or jthread dispatch.
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

    // Separate position pass, after velocities/impulse cache have been
    // updated. Existing manifold depths are reused within this pass.
    // World-facing position pass after velocity iterations and cache
    // storage. It does not generate new collision witnesses or replace
    // CCD for fast-moving objects.
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
