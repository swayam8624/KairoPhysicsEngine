from pathlib import Path
import re

# Narrowphase: deterministic multi-point box manifolds.
p = Path('Narrowphase.cppm')
s = p.read_text()
if '#include <array>\n' not in s:
    s = s.replace('#include <algorithm>\n', '#include <algorithm>\n#include <array>\n', 1)

marker = '''    [[nodiscard]]
    inline AABBf WorldBoundsToColliderLocal('''
if marker not in s:
    raise SystemExit('WorldBoundsToColliderLocal marker not found')

helpers = r'''    [[nodiscard]]
    inline std::array<Vec3f, 8> BoxVertices(
        const OrientedBoxFrame& box)
    {
        std::array<Vec3f, 8> vertices{};
        std::size_t index = 0u;
        for (int x = -1; x <= 1; x += 2)
        {
            for (int y = -1; y <= 1; y += 2)
            {
                for (int z = -1; z <= 1; z += 2)
                {
                    vertices[index++] =
                        box.Center +
                        box.Axes[0] * (box.HalfExtents.x * static_cast<float>(x)) +
                        box.Axes[1] * (box.HalfExtents.y * static_cast<float>(y)) +
                        box.Axes[2] * (box.HalfExtents.z * static_cast<float>(z));
                }
            }
        }
        return vertices;
    }

    [[nodiscard]]
    inline bool PointInsideBox(
        const OrientedBoxFrame& box,
        const Vec3f& point,
        float tolerance = 1.0e-4f) noexcept
    {
        const Vec3f delta = point - box.Center;
        for (std::size_t axis = 0u; axis < 3u; ++axis)
        {
            if (std::abs(Dot(delta, box.Axes[axis])) >
                BoxExtentAt(box.HalfExtents, axis) + tolerance)
            {
                return false;
            }
        }
        return true;
    }

    inline void AppendUniqueContact(
        std::vector<ContactPoint>& points,
        const ContactPoint& candidate)
    {
        constexpr float mergeDistanceSq = 1.0e-8f;
        for (ContactPoint& existing : points)
        {
            if ((existing.Position - candidate.Position).LengthSquared() <= mergeDistanceSq &&
                Dot(SafeNormalize(existing.Normal, Vec3f::Up()),
                    SafeNormalize(candidate.Normal, Vec3f::Up())) > 0.99f)
            {
                if (candidate.PenetrationDepth > existing.PenetrationDepth)
                {
                    existing = candidate;
                }
                return;
            }
        }
        points.push_back(candidate);
    }

    inline void ReduceContactPoints(
        std::vector<ContactPoint>& points,
        std::size_t maxPoints = 4u)
    {
        if (points.size() <= maxPoints)
        {
            return;
        }

        std::vector<ContactPoint> reduced;
        reduced.reserve(maxPoints);
        std::vector<bool> selected(points.size(), false);

        std::size_t deepest = 0u;
        for (std::size_t i = 1u; i < points.size(); ++i)
        {
            if (points[i].PenetrationDepth > points[deepest].PenetrationDepth)
            {
                deepest = i;
            }
        }
        reduced.push_back(points[deepest]);
        selected[deepest] = true;

        while (reduced.size() < maxPoints)
        {
            std::size_t best = points.size();
            float bestSpread = -1.0f;
            for (std::size_t i = 0u; i < points.size(); ++i)
            {
                if (selected[i])
                {
                    continue;
                }

                float nearestSq = std::numeric_limits<float>::max();
                for (const ContactPoint& chosen : reduced)
                {
                    nearestSq = std::min(
                        nearestSq,
                        (points[i].Position - chosen.Position).LengthSquared());
                }

                if (nearestSq > bestSpread)
                {
                    bestSpread = nearestSq;
                    best = i;
                }
            }

            if (best == points.size())
            {
                break;
            }
            selected[best] = true;
            reduced.push_back(points[best]);
        }

        points = std::move(reduced);
    }

    inline void BuildBoxBoxContacts(
        ContactManifold& manifold,
        const OrientedBoxFrame& boxA,
        const OrientedBoxFrame& boxB,
        const Vec3f& collisionNormal,
        float penetration)
    {
        const Vec3f normal =
            SafeNormalize(collisionNormal, Vec3f::UnitX());
        const float halfPenetration = penetration * 0.5f;

        for (const Vec3f& vertex : BoxVertices(boxA))
        {
            if (PointInsideBox(boxB, vertex))
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex - normal * halfPenetration,
                        normal,
                        penetration));
            }
        }

        for (const Vec3f& vertex : BoxVertices(boxB))
        {
            if (PointInsideBox(boxA, vertex))
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex + normal * halfPenetration,
                        normal,
                        penetration));
            }
        }

        if (manifold.Points.empty())
        {
            const Vec3f pointA = SupportPoint(boxA, normal);
            const Vec3f pointB = SupportPoint(boxB, -normal);
            manifold.Points.push_back(
                MakeContactPoint(
                    (pointA + pointB) * 0.5f,
                    normal,
                    penetration));
        }

        ReduceContactPoints(manifold.Points, 4u);
    }

    inline void BuildBoxPlaneContacts(
        ContactManifold& manifold,
        const OrientedBoxFrame& box,
        const PlaneCollider& plane)
    {
        const Vec3f planeNormal =
            SafeNormalize(plane.Normal, Vec3f::Up());

        for (const Vec3f& vertex : BoxVertices(box))
        {
            const float signedDistance =
                Dot(planeNormal, vertex) + plane.Distance;
            if (signedDistance < 0.0f)
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex - planeNormal * (signedDistance * 0.5f),
                        -planeNormal,
                        -signedDistance));
            }
        }

        if (manifold.Points.empty())
        {
            const float projectedRadius = ProjectBoxRadius(box, planeNormal);
            const float signedCenter = Dot(planeNormal, box.Center) + plane.Distance;
            const float penetration = projectedRadius - signedCenter;
            if (penetration > 0.0f)
            {
                manifold.Points.push_back(
                    MakeContactPoint(
                        SupportPoint(box, -planeNormal),
                        -planeNormal,
                        penetration));
            }
        }

        ReduceContactPoints(manifold.Points, 4u);
    }

'''
if 'inline std::array<Vec3f, 8> BoxVertices(' not in s:
    s = s.replace(marker, helpers + marker, 1)

# Oriented box section.
start = s.index('        if (const auto* boxA = std::get_if<BoxCollider>(&colliderA.Shape))')
end = s.index('        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))', start)
seg = s[start:end]
old = '''                const Vec3f pointA =
                    SupportPoint(frameA, normal);

                const Vec3f pointB =
                    SupportPoint(frameB, -normal);

                manifold.Points.push_back(
                    MakeContactPoint(
                        (pointA + pointB) * 0.5f,
                        normal,
                        penetration));

                return manifold;'''
new = '''                BuildBoxBoxContacts(
                    manifold,
                    frameA,
                    frameB,
                    normal,
                    penetration);
                return manifold;'''
if old not in seg:
    raise SystemExit('oriented box-box contact snippet not found')
seg = seg.replace(old, new, 1)

plane_old = '''                const float projectedRadius =
                    ProjectBoxRadius(frameA, planeB->Normal);

                const float signedDistance =
                    Dot(planeB->Normal, frameA.Center) + planeB->Distance;

                const float penetration =
                    projectedRadius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        SupportPoint(frameA, -planeB->Normal),
                        -planeB->Normal,
                        penetration));

                return manifold;'''
plane_new = '''                BuildBoxPlaneContacts(manifold, frameA, *planeB);
                if (manifold.Points.empty())
                {
                    return std::nullopt;
                }
                return manifold;'''
if plane_old not in seg:
    raise SystemExit('oriented box-plane snippet not found')
seg = seg.replace(plane_old, plane_new, 1)

mixed = '''
            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB
                {
                    centerB,
                    { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                    boxB->HalfExtents
                };
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(frameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, frameA, frameB, normal, penetration);
                return manifold;
            }

'''
plane_marker = '            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))'
if 'std::get_if<AABBCollider>(&colliderB.Shape)' not in seg:
    seg = seg.replace(plane_marker, mixed + plane_marker, 1)
s = s[:start] + seg + s[end:]

# Axis-aligned box section.
start = s.index('        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))')
end = s.index('        if (std::holds_alternative<PlaneCollider>(colliderA.Shape)', start)
seg = s[start:end]
opening = '''        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))
        {
'''
frame_decl = '''        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))
        {
            const OrientedBoxFrame axisAlignedFrameA
            {
                centerA,
                { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                boxA->HalfExtents
            };
'''
if opening not in seg:
    raise SystemExit('AABB section opening not found')
seg = seg.replace(opening, frame_decl, 1)

pattern = re.compile(r'''            if \(const auto\* boxB = std::get_if<AABBCollider>\(&colliderB\.Shape\)\)\n            \{.*?\n                return manifold;\n            \}\n''', re.S)
m = pattern.search(seg)
if not m:
    raise SystemExit('AABB-AABB block not found')
aabb_new = '''            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB
                {
                    centerB,
                    { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                    boxB->HalfExtents
                };
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(axisAlignedFrameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, axisAlignedFrameA, frameB, normal, penetration);
                return manifold;
            }
'''
seg = seg[:m.start()] + aabb_new + seg[m.end():]

mixed_aabb = '''            if (const auto* boxB = std::get_if<BoxCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB =
                    WorldBoxFrame(bodyB, colliderB, boxB->HalfExtents);
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(axisAlignedFrameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, axisAlignedFrameA, frameB, normal, penetration);
                return manifold;
            }

'''
aabb_marker = '            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))'
if 'std::get_if<BoxCollider>(&colliderB.Shape)' not in seg:
    seg = seg.replace(aabb_marker, mixed_aabb + aabb_marker, 1)

aabb_plane_old = '''                const float projectedRadius =
                    std::abs(planeB->Normal.x) * boxA->HalfExtents.x +
                    std::abs(planeB->Normal.y) * boxA->HalfExtents.y +
                    std::abs(planeB->Normal.z) * boxA->HalfExtents.z;

                const float signedDistance =
                    Dot(planeB->Normal, centerA) + planeB->Distance;

                const float penetration =
                    projectedRadius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        centerA - planeB->Normal * projectedRadius,
                        -planeB->Normal,
                        penetration));

                return manifold;'''
aabb_plane_new = '''                BuildBoxPlaneContacts(manifold, axisAlignedFrameA, *planeB);
                if (manifold.Points.empty())
                {
                    return std::nullopt;
                }
                return manifold;'''
if aabb_plane_old not in seg:
    raise SystemExit('AABB-plane snippet not found')
seg = seg.replace(aabb_plane_old, aabb_plane_new, 1)
s = s[:start] + seg + s[end:]
p.write_text(s)

# PhysicsWorld: geometry-based persistent warm-start cache.
p = Path('PhysicsWorld.cppm')
s = p.read_text()
old_decl = '''        struct ContactCacheEntry final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            std::uint32_t PointIndex = 0;
            float NormalImpulse = 0.0f;
            float TangentImpulse = 0.0f;
        };'''
new_decl = '''        struct ContactCacheEntry final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            Vec3f LocalAnchorA = Vec3f::Zero();
            Vec3f LocalAnchorB = Vec3f::Zero();
            Vec3f Normal = Vec3f::Up();
            float NormalImpulse = 0.0f;
            float TangentImpulse = 0.0f;
        };'''
if old_decl not in s:
    raise SystemExit('ContactCacheEntry declaration not found')
s = s.replace(old_decl, new_decl, 1)

cache_start = s.index('        [[nodiscard]]\n        static bool SameCacheKey(')
class_close = s.rfind('    };\n}')
if class_close < cache_start:
    raise SystemExit('PhysicsWorld class close not found')
cache_impl = r'''        [[nodiscard]]
        static bool SameCachePair(
            const ContactCacheEntry& entry,
            const ContactManifold& manifold) noexcept
        {
            return entry.BodyA == manifold.BodyA &&
                entry.BodyB == manifold.BodyB &&
                entry.ColliderA == manifold.ColliderA &&
                entry.ColliderB == manifold.ColliderB;
        }

        [[nodiscard]]
        static Vec3f ContactAnchorInBodySpace(
            const RigidBody& body,
            const Vec3f& worldPoint)
        {
            return Rotate(
                body.State.Rotation.Conjugate(),
                worldPoint - body.State.Position);
        }

        void RestoreContactCache()
        {
            std::vector<bool> used(m_ContactCache.size(), false);
            constexpr float anchorMatchDistanceSq = 0.01f;
            constexpr float minimumNormalAgreement = 0.80f;

            for (ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.BodyA >= m_Bodies.size() ||
                    manifold.BodyB >= m_Bodies.size())
                {
                    continue;
                }

                const RigidBody& bodyA = m_Bodies.at(manifold.BodyA);
                const RigidBody& bodyB = m_Bodies.at(manifold.BodyB);

                for (ContactPoint& point : manifold.Points)
                {
                    const Vec3f localAnchorA = ContactAnchorInBodySpace(bodyA, point.Position);
                    const Vec3f localAnchorB = ContactAnchorInBodySpace(bodyB, point.Position);
                    const Vec3f normal = SafeNormalize(point.Normal, Vec3f::Up());

                    std::size_t best = m_ContactCache.size();
                    float bestError = std::numeric_limits<float>::max();
                    for (std::size_t index = 0u; index < m_ContactCache.size(); ++index)
                    {
                        if (used[index]) continue;
                        const ContactCacheEntry& entry = m_ContactCache[index];
                        if (!SameCachePair(entry, manifold) ||
                            Dot(normal, entry.Normal) < minimumNormalAgreement)
                        {
                            continue;
                        }

                        const float errorA = (localAnchorA - entry.LocalAnchorA).LengthSquared();
                        const float errorB = (localAnchorB - entry.LocalAnchorB).LengthSquared();
                        if (errorA > anchorMatchDistanceSq || errorB > anchorMatchDistanceSq)
                        {
                            continue;
                        }

                        const float totalError = errorA + errorB;
                        if (totalError < bestError)
                        {
                            bestError = totalError;
                            best = index;
                        }
                    }

                    if (best != m_ContactCache.size())
                    {
                        point.NormalImpulse = m_ContactCache[best].NormalImpulse;
                        point.TangentImpulse = m_ContactCache[best].TangentImpulse;
                        used[best] = true;
                    }
                }
            }
        }

        void StoreContactCache()
        {
            m_ContactCache.clear();
            for (const ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.IsTrigger ||
                    manifold.BodyA >= m_Bodies.size() ||
                    manifold.BodyB >= m_Bodies.size())
                {
                    continue;
                }

                const RigidBody& bodyA = m_Bodies.at(manifold.BodyA);
                const RigidBody& bodyB = m_Bodies.at(manifold.BodyB);
                for (const ContactPoint& point : manifold.Points)
                {
                    if (point.NormalImpulse <= 0.0f &&
                        std::abs(point.TangentImpulse) <= 1.0e-8f)
                    {
                        continue;
                    }

                    m_ContactCache.push_back(
                        ContactCacheEntry
                        {
                            manifold.BodyA,
                            manifold.BodyB,
                            manifold.ColliderA,
                            manifold.ColliderB,
                            ContactAnchorInBodySpace(bodyA, point.Position),
                            ContactAnchorInBodySpace(bodyB, point.Position),
                            SafeNormalize(point.Normal, Vec3f::Up()),
                            point.NormalImpulse,
                            point.TangentImpulse
                        });
                }
            }

            const auto lessVec3 =
                [](const Vec3f& lhs, const Vec3f& rhs) noexcept
                {
                    if (lhs.x != rhs.x) return lhs.x < rhs.x;
                    if (lhs.y != rhs.y) return lhs.y < rhs.y;
                    return lhs.z < rhs.z;
                };

            std::sort(
                m_ContactCache.begin(),
                m_ContactCache.end(),
                [&lessVec3](const ContactCacheEntry& lhs, const ContactCacheEntry& rhs)
                {
                    if (lhs.BodyA != rhs.BodyA) return lhs.BodyA < rhs.BodyA;
                    if (lhs.BodyB != rhs.BodyB) return lhs.BodyB < rhs.BodyB;
                    if (lhs.ColliderA != rhs.ColliderA) return lhs.ColliderA < rhs.ColliderA;
                    if (lhs.ColliderB != rhs.ColliderB) return lhs.ColliderB < rhs.ColliderB;
                    if (lessVec3(lhs.LocalAnchorA, rhs.LocalAnchorA)) return true;
                    if (lessVec3(rhs.LocalAnchorA, lhs.LocalAnchorA)) return false;
                    return lessVec3(lhs.LocalAnchorB, rhs.LocalAnchorB);
                });
        }
'''
s = s[:cache_start] + cache_impl + s[class_close:]
p.write_text(s)

# Tests.
p = Path('tests/PhysicsEngineTests.cpp')
s = p.read_text()
marker = 'TEST_CASE("Triangle mesh validation builds a SAH BVH and rejects invalid data",'
if marker not in s:
    raise SystemExit('test insertion marker not found')
tests = r'''TEST_CASE("Box plane contact produces a stable four-point face manifold",
    "[PhysicsEngine][Manifold][Box][Plane]")
{
    const RigidBody boxBody =
        MakeRigidBody(0, DynamicBoxBody(Vec3f{ 0.0f, 0.45f, 0.0f }));
    const RigidBody planeBody = MakeRigidBody(1, StaticBody());
    const Collider box =
        MakeCollider(0, 0, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });
    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto contact = CollidePair(boxBody, box, planeBody, plane);
    REQUIRE(contact.has_value());
    REQUIRE(contact->Points.size() == 4u);
    for (const ContactPoint& point : contact->Points)
    {
        CHECK(point.Normal.y < -0.99f);
        CHECK(point.PenetrationDepth == Catch::Approx(0.05f).margin(1.0e-4f));
    }
}

TEST_CASE("Oriented box and AABB use the shared SAT manifold path",
    "[PhysicsEngine][Manifold][Box][AABB]")
{
    RigidBodyDesc orientedDesc = DynamicBoxBody(Vec3f::Zero());
    orientedDesc.State.Rotation = RotationAroundZ(0.15f);
    const RigidBody orientedBody = MakeRigidBody(0, orientedDesc);
    const RigidBody axisBody =
        MakeRigidBody(1, StaticBody(Vec3f{ 0.80f, 0.0f, 0.0f }));
    const Collider oriented =
        MakeCollider(0, 0, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });
    const Collider axis =
        MakeCollider(1, 1, AABBCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const auto forward = CollidePair(orientedBody, oriented, axisBody, axis);
    const auto reverse = CollidePair(axisBody, axis, orientedBody, oriented);
    REQUIRE(forward.has_value());
    REQUIRE(reverse.has_value());
    REQUIRE_FALSE(forward->Points.empty());
    REQUIRE_FALSE(reverse->Points.empty());
    CHECK(forward->Points.size() <= 4u);
    CHECK(reverse->Points.size() <= 4u);
    CHECK(Dot(forward->Points.front().Normal, reverse->Points.front().Normal) < -0.95f);
}

TEST_CASE("Resting box manifolds retain multiple warm-started impulses",
    "[PhysicsEngine][Manifold][WarmStart]")
{
    PhysicsWorld world;
    const BodyID planeBody = world.CreateRigidBody(StaticBody());
    world.AddCollider(planeBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    const BodyID boxBody = world.CreateRigidBody(
        DynamicBoxBody(Vec3f{ 0.0f, 0.48f, 0.0f }));
    world.AddCollider(boxBody, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    world.Step(1.0f / 60.0f);
    REQUIRE_FALSE(world.Contacts().empty());
    REQUIRE(world.Contacts().front().Points.size() >= 2u);

    world.Step(1.0f / 60.0f);
    REQUIRE_FALSE(world.Contacts().empty());
    REQUIRE(world.Contacts().front().Points.size() >= 2u);

    std::size_t warmedPoints = 0u;
    for (const ContactPoint& point : world.Contacts().front().Points)
    {
        if (point.NormalImpulse > 1.0e-6f) ++warmedPoints;
    }
    CHECK(warmedPoints >= 2u);
}

'''
if 'Box plane contact produces a stable four-point face manifold' not in s:
    s = s.replace(marker, tests + marker, 1)
p.write_text(s)

# README.
p = Path('README.md')
s = p.read_text()
s = s.replace(
    'Sequential impulse solver with warm-started normal/friction impulses',
    'Sequential impulse solver with geometrically matched persistent multi-point manifolds and warm-started normal/friction impulses')
s = s.replace(
    'Sphere-sphere, sphere-plane, sphere-box, AABB-AABB, AABB-plane, box-box SAT, and box-plane contacts',
    'Sphere-sphere, sphere-plane, sphere-box, AABB-AABB, AABB-plane, box-box, mixed box-AABB SAT, and box-plane contacts')
s = s.replace('Persistent multi-point contact manifolds\n', '')
s = s.replace(
    'dynamic triangle-mesh motion, persistent multi-point manifolds, general CCD, joints,\nand island solving',
    'general CCD, joints, and island solving')
s = s.replace(
    'Narrowphase          exact V1 contact generation and box SAT',
    'Narrowphase          exact contact generation, box SAT, and deterministic four-point face manifolds')
p.write_text(s)
