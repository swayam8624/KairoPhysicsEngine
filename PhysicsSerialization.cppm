module;

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Serialization;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.Material;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Broadphase;
import Kairo.Foundation.PhysicsEngine.Joint;
import Kairo.Foundation.PhysicsEngine.World;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    inline constexpr std::uint32_t PhysicsSnapshotFileVersion = 1u;
    inline constexpr std::size_t PhysicsSnapshotMaxBytes = 256u * 1024u * 1024u;

    namespace serialization_detail
    {
        inline constexpr std::array<std::uint8_t, 8> SnapshotMagic{
            'K', 'P', 'H', 'Y', 'S', '0', '1', '\n' };
        inline constexpr std::uint32_t MaxRecords = 1'000'000u;
        inline constexpr std::uint32_t MaxShapeElements = 16'000'000u;

        class Writer final
        {
        public:
            std::vector<std::uint8_t> Bytes;

            void U8(std::uint8_t value) { Bytes.push_back(value); }
            void Bool(bool value) { U8(value ? 1u : 0u); }
            void U32(std::uint32_t value)
            {
                for (int shift = 0; shift < 32; shift += 8)
                    U8(static_cast<std::uint8_t>((value >> shift) & 0xFFu));
            }
            void U64(std::uint64_t value)
            {
                for (int shift = 0; shift < 64; shift += 8)
                    U8(static_cast<std::uint8_t>((value >> shift) & 0xFFu));
            }
            void Float(float value) { U32(std::bit_cast<std::uint32_t>(value)); }
            void Vec3(const Vec3f& v) { Float(v.x); Float(v.y); Float(v.z); }
            void Quat(const Quaternionf& q) { Float(q.x); Float(q.y); Float(q.z); Float(q.w); }
            void Mat3(const Matrix3f& m)
            {
                for (std::size_t i = 0u; i < Matrix3f::ElementCount; ++i) Float(m[i]);
            }
            void Raw(const std::uint8_t* data, std::size_t size)
            {
                if (size > PhysicsSnapshotMaxBytes || Bytes.size() > PhysicsSnapshotMaxBytes - size)
                    throw std::length_error("Physics snapshot exceeds the maximum byte size.");
                Bytes.insert(Bytes.end(), data, data + size);
            }
            template<std::size_t N> void Raw(const std::array<std::uint8_t, N>& data)
            { Raw(data.data(), N); }
            void Count(std::size_t count, std::uint32_t limit = MaxRecords)
            {
                if (count > limit || count > std::numeric_limits<std::uint32_t>::max())
                    throw std::length_error("Physics snapshot record count exceeds the format limit.");
                U32(static_cast<std::uint32_t>(count));
            }
        };

        class Reader final
        {
        public:
            explicit Reader(const std::vector<std::uint8_t>& bytes) : m_Bytes(bytes) {}

            std::uint8_t U8()
            {
                Require(1u);
                return m_Bytes[m_Offset++];
            }
            bool Bool()
            {
                const std::uint8_t value = U8();
                if (value > 1u) throw std::runtime_error("Invalid serialized boolean.");
                return value != 0u;
            }
            std::uint32_t U32()
            {
                Require(4u);
                std::uint32_t value = 0u;
                for (int shift = 0; shift < 32; shift += 8)
                    value |= static_cast<std::uint32_t>(m_Bytes[m_Offset++]) << shift;
                return value;
            }
            std::uint64_t U64()
            {
                Require(8u);
                std::uint64_t value = 0u;
                for (int shift = 0; shift < 64; shift += 8)
                    value |= static_cast<std::uint64_t>(m_Bytes[m_Offset++]) << shift;
                return value;
            }
            float Float() { return std::bit_cast<float>(U32()); }
            Vec3f Vec3() { return { Float(), Float(), Float() }; }
            Quaternionf Quat() { return { Float(), Float(), Float(), Float() }; }
            Matrix3f Mat3()
            {
                Matrix3f result;
                for (std::size_t i = 0u; i < Matrix3f::ElementCount; ++i) result[i] = Float();
                return result;
            }
            std::uint32_t Count(std::uint32_t limit = MaxRecords)
            {
                const std::uint32_t count = U32();
                if (count > limit) throw std::runtime_error("Serialized record count exceeds the format limit.");
                return count;
            }
            void Magic(const std::array<std::uint8_t, 8>& expected)
            {
                for (std::uint8_t byte : expected)
                    if (U8() != byte) throw std::runtime_error("Physics snapshot magic is invalid.");
            }
            void RequireEnd() const
            {
                if (m_Offset != m_Bytes.size())
                    throw std::runtime_error("Physics snapshot contains trailing bytes.");
            }
        private:
            const std::vector<std::uint8_t>& m_Bytes;
            std::size_t m_Offset = 0u;
            void Require(std::size_t size) const
            {
                if (size > m_Bytes.size() - std::min(m_Offset, m_Bytes.size()))
                    throw std::runtime_error("Physics snapshot is truncated.");
            }
        };

        inline void WriteBodyType(Writer& w, BodyType type)
        {
            switch (type)
            {
            case BodyType::Static: w.U8(0u); return;
            case BodyType::Kinematic: w.U8(1u); return;
            case BodyType::Dynamic: w.U8(2u); return;
            }
            throw std::invalid_argument("Cannot serialize invalid BodyType.");
        }
        inline BodyType ReadBodyType(Reader& r)
        {
            switch (r.U8())
            {
            case 0u: return BodyType::Static;
            case 1u: return BodyType::Kinematic;
            case 2u: return BodyType::Dynamic;
            default: throw std::runtime_error("Serialized BodyType is invalid.");
            }
        }
        inline void WriteResponse(Writer& w, CollisionResponse response)
        {
            switch (response)
            {
            case CollisionResponse::Ignore: w.U8(0u); return;
            case CollisionResponse::Trigger: w.U8(1u); return;
            case CollisionResponse::Block: w.U8(2u); return;
            }
            throw std::invalid_argument("Cannot serialize invalid CollisionResponse.");
        }
        inline CollisionResponse ReadResponse(Reader& r)
        {
            switch (r.U8())
            {
            case 0u: return CollisionResponse::Ignore;
            case 1u: return CollisionResponse::Trigger;
            case 2u: return CollisionResponse::Block;
            default: throw std::runtime_error("Serialized CollisionResponse is invalid.");
            }
        }
        inline void WriteCCD(Writer& w, CollisionDetectionMode mode)
        {
            switch (mode)
            {
            case CollisionDetectionMode::Discrete: w.U8(0u); return;
            case CollisionDetectionMode::Continuous: w.U8(1u); return;
            }
            throw std::invalid_argument("Cannot serialize invalid CCD mode.");
        }
        inline CollisionDetectionMode ReadCCD(Reader& r)
        {
            switch (r.U8())
            {
            case 0u: return CollisionDetectionMode::Discrete;
            case 1u: return CollisionDetectionMode::Continuous;
            default: throw std::runtime_error("Serialized CCD mode is invalid.");
            }
        }

        inline void WriteSettings(Writer& w, const PhysicsStepSettings& s)
        {
            w.Float(s.GravityScale); w.U32(s.VelocityIterations); w.U32(s.PositionIterations);
            w.Float(s.Baumgarte); w.Float(s.Slop); w.Float(s.MaxPositionCorrection);
            w.Bool(s.EnableParallelIslands); w.U32(s.ParallelIslandMinCount);
            w.Bool(s.EnableSleeping); w.Float(s.SleepLinearSpeed);
            w.Float(s.SleepAngularSpeed); w.Float(s.SleepTime);
        }
        inline PhysicsStepSettings ReadSettings(Reader& r)
        {
            PhysicsStepSettings s;
            s.GravityScale = r.Float(); s.VelocityIterations = r.U32();
            s.PositionIterations = r.U32(); s.Baumgarte = r.Float();
            s.Slop = r.Float(); s.MaxPositionCorrection = r.Float();
            s.EnableParallelIslands = r.Bool(); s.ParallelIslandMinCount = r.U32();
            s.EnableSleeping = r.Bool(); s.SleepLinearSpeed = r.Float();
            s.SleepAngularSpeed = r.Float(); s.SleepTime = r.Float();
            return s;
        }

        inline void WriteMotionState(Writer& w, const MotionState& s)
        { w.Vec3(s.Position); w.Quat(s.Rotation); w.Vec3(s.LinearVelocity); w.Vec3(s.AngularVelocity); }
        inline MotionState ReadMotionState(Reader& r)
        { return { r.Vec3(), r.Quat(), r.Vec3(), r.Vec3() }; }
        inline void WriteMass(Writer& w, const MassProperties& m)
        {
            w.Float(m.Mass); w.Float(m.InverseMass); w.Vec3(m.LocalCenterOfMass);
            w.Mat3(m.LocalInertiaTensor); w.Mat3(m.LocalInverseInertiaTensor);
        }
        inline MassProperties ReadMass(Reader& r)
        { return { r.Float(), r.Float(), r.Vec3(), r.Mat3(), r.Mat3() }; }

        inline void WriteBody(Writer& w, const RigidBody& b)
        {
            w.U32(b.ID); w.Bool(b.Active); WriteBodyType(w, b.Type); WriteMotionState(w, b.State);
            WriteMass(w, b.Mass); w.Vec3(b.Forces.Force); w.Vec3(b.Forces.Torque);
            w.Bool(b.EnableGravity); w.Float(b.GravityScale); w.Float(b.LinearDamping);
            w.Float(b.AngularDamping); w.Float(b.MaxLinearSpeed); w.Float(b.MaxAngularSpeed);
            WriteCCD(w, b.CollisionDetection); w.Bool(b.AllowSleeping); w.Bool(b.Sleeping);
            w.Float(b.SleepTimer);
        }
        inline RigidBody ReadBody(Reader& r)
        {
            RigidBody b;
            b.ID = r.U32(); b.Active = r.Bool(); b.Type = ReadBodyType(r);
            b.State = ReadMotionState(r); b.Mass = ReadMass(r);
            b.Forces.Force = r.Vec3(); b.Forces.Torque = r.Vec3();
            b.EnableGravity = r.Bool(); b.GravityScale = r.Float();
            b.LinearDamping = r.Float(); b.AngularDamping = r.Float();
            b.MaxLinearSpeed = r.Float(); b.MaxAngularSpeed = r.Float();
            b.CollisionDetection = ReadCCD(r); b.AllowSleeping = r.Bool();
            b.Sleeping = r.Bool(); b.SleepTimer = r.Float();
            return b;
        }

        inline void WriteMaterial(Writer& w, const PhysicsMaterial& m)
        { w.Float(m.Restitution); w.Float(m.StaticFriction); w.Float(m.DynamicFriction); }
        inline PhysicsMaterial ReadMaterial(Reader& r)
        { return { r.Float(), r.Float(), r.Float() }; }

        inline void WriteShape(Writer& w, const ColliderShape& shape)
        {
            std::visit([&](const auto& value)
            {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, SphereCollider>)
                { w.U8(0u); w.Float(value.Radius); }
                else if constexpr (std::is_same_v<T, CapsuleCollider>)
                { w.U8(1u); w.Float(value.Radius); w.Float(value.HalfHeight); }
                else if constexpr (std::is_same_v<T, PlaneCollider>)
                { w.U8(2u); w.Vec3(value.Normal); w.Float(value.Distance); }
                else if constexpr (std::is_same_v<T, AABBCollider>)
                { w.U8(3u); w.Vec3(value.HalfExtents); }
                else if constexpr (std::is_same_v<T, BoxCollider>)
                { w.U8(4u); w.Vec3(value.HalfExtents); }
                else if constexpr (std::is_same_v<T, ConvexHullCollider>)
                {
                    w.U8(5u); w.Count(value.Vertices.size(), MaxShapeElements);
                    for (const Vec3f& vertex : value.Vertices) w.Vec3(vertex);
                    w.Count(value.Faces.size(), MaxShapeElements);
                    for (const auto& face : value.Faces)
                    { w.U32(face[0]); w.U32(face[1]); w.U32(face[2]); }
                }
                else if constexpr (std::is_same_v<T, TriangleMeshCollider>)
                {
                    w.U8(6u); w.Count(value.Vertices.size(), MaxShapeElements);
                    for (const Vec3f& vertex : value.Vertices) w.Vec3(vertex);
                    w.Count(value.Triangles.size(), MaxShapeElements);
                    for (const auto& tri : value.Triangles)
                    { w.U32(tri[0]); w.U32(tri[1]); w.U32(tri[2]); }
                    w.Float(value.SurfaceThickness); w.Bool(value.DoubleSided);
                }
            }, shape);
        }
        inline ColliderShape ReadShape(Reader& r)
        {
            switch (r.U8())
            {
            case 0u: return SphereCollider{ r.Float() };
            case 1u: return CapsuleCollider{ r.Float(), r.Float() };
            case 2u: return PlaneCollider{ r.Vec3(), r.Float() };
            case 3u: return AABBCollider{ r.Vec3() };
            case 4u: return BoxCollider{ r.Vec3() };
            case 5u:
            {
                ConvexHullCollider hull;
                const auto vc = r.Count(MaxShapeElements); hull.Vertices.reserve(vc);
                for (std::uint32_t i = 0; i < vc; ++i) hull.Vertices.push_back(r.Vec3());
                const auto fc = r.Count(MaxShapeElements); hull.Faces.reserve(fc);
                for (std::uint32_t i = 0; i < fc; ++i)
                    hull.Faces.push_back({ r.U32(), r.U32(), r.U32() });
                return hull;
            }
            case 6u:
            {
                TriangleMeshCollider mesh;
                const auto vc = r.Count(MaxShapeElements); mesh.Vertices.reserve(vc);
                for (std::uint32_t i = 0; i < vc; ++i) mesh.Vertices.push_back(r.Vec3());
                const auto tc = r.Count(MaxShapeElements); mesh.Triangles.reserve(tc);
                for (std::uint32_t i = 0; i < tc; ++i)
                    mesh.Triangles.push_back({ r.U32(), r.U32(), r.U32() });
                mesh.SurfaceThickness = r.Float(); mesh.DoubleSided = r.Bool();
                return mesh;
            }
            default: throw std::runtime_error("Serialized collider shape tag is invalid.");
            }
        }

        inline void WriteCollider(Writer& w, const Collider& c)
        {
            w.U32(c.ID); w.Bool(c.Active); w.U32(c.Body); w.Vec3(c.LocalCenter);
            w.Quat(c.LocalRotation); WriteShape(w, c.Shape); WriteMaterial(w, c.Material);
            w.U32(c.BelongsTo); w.U32(c.CollidesWith); w.Bool(c.IsTrigger); w.U32(c.LayerMask);
        }
        inline Collider ReadCollider(Reader& r)
        {
            Collider c;
            c.ID = r.U32(); c.Active = r.Bool(); c.Body = r.U32(); c.LocalCenter = r.Vec3();
            c.LocalRotation = r.Quat(); c.Shape = ReadShape(r); c.Material = ReadMaterial(r);
            c.BelongsTo = r.U32(); c.CollidesWith = r.U32(); c.IsTrigger = r.Bool();
            c.LayerMask = r.U32(); return c;
        }

        inline void WriteJoint(Writer& w, const Joint& joint)
        {
            w.U32(joint.ID); w.Bool(joint.Active); w.U32(joint.BodyA); w.U32(joint.BodyB);
            w.Bool(joint.CollideConnected);
            std::visit([&](const auto& c)
            {
                using T = std::decay_t<decltype(c)>;
                if constexpr (std::is_same_v<T, DistanceJoint>)
                { w.U8(0u); w.Vec3(c.LocalAnchorA); w.Vec3(c.LocalAnchorB); w.Float(c.RestLength); }
                else if constexpr (std::is_same_v<T, BallSocketJoint>)
                { w.U8(1u); w.Vec3(c.LocalAnchorA); w.Vec3(c.LocalAnchorB); }
                else if constexpr (std::is_same_v<T, FixedJoint>)
                { w.U8(2u); w.Vec3(c.LocalAnchorA); w.Vec3(c.LocalAnchorB); w.Quat(c.ReferenceRotation); }
                else if constexpr (std::is_same_v<T, HingeJoint>)
                { w.U8(3u); w.Vec3(c.LocalAnchorA); w.Vec3(c.LocalAnchorB); w.Vec3(c.LocalAxisA); w.Vec3(c.LocalAxisB); }
            }, joint.Constraint);
        }
        inline Joint ReadJoint(Reader& r)
        {
            Joint joint;
            joint.ID = r.U32(); joint.Active = r.Bool(); joint.BodyA = r.U32(); joint.BodyB = r.U32();
            joint.CollideConnected = r.Bool();
            switch (r.U8())
            {
            case 0u: joint.Constraint = DistanceJoint{ r.Vec3(), r.Vec3(), r.Float() }; break;
            case 1u: joint.Constraint = BallSocketJoint{ r.Vec3(), r.Vec3() }; break;
            case 2u: joint.Constraint = FixedJoint{ r.Vec3(), r.Vec3(), r.Quat() }; break;
            case 3u: joint.Constraint = HingeJoint{ r.Vec3(), r.Vec3(), r.Vec3(), r.Vec3() }; break;
            default: throw std::runtime_error("Serialized joint tag is invalid.");
            }
            return joint;
        }

        inline void WriteContactPoint(Writer& w, const ContactPoint& p)
        { w.Vec3(p.Position); w.Vec3(p.Normal); w.Float(p.PenetrationDepth); w.Float(p.NormalImpulse); w.Float(p.TangentImpulse); }
        inline ContactPoint ReadContactPoint(Reader& r)
        { return { r.Vec3(), r.Vec3(), r.Float(), r.Float(), r.Float() }; }
        inline void WriteManifold(Writer& w, const ContactManifold& m)
        {
            w.U32(m.BodyA); w.U32(m.BodyB); w.U32(m.ColliderA); w.U32(m.ColliderB); w.Bool(m.IsTrigger);
            w.Count(m.Points.size(), 64u); for (const auto& p : m.Points) WriteContactPoint(w, p);
        }
        inline ContactManifold ReadManifold(Reader& r)
        {
            ContactManifold m; m.BodyA = r.U32(); m.BodyB = r.U32();
            m.ColliderA = r.U32(); m.ColliderB = r.U32(); m.IsTrigger = r.Bool();
            const auto count = r.Count(64u); m.Points.reserve(count);
            for (std::uint32_t i = 0; i < count; ++i) m.Points.push_back(ReadContactPoint(r));
            return m;
        }

        inline std::vector<std::uint8_t> ReadFile(const std::filesystem::path& path)
        {
            std::error_code ec;
            const std::uintmax_t size = std::filesystem::file_size(path, ec);
            if (ec) throw std::runtime_error("Failed to stat physics snapshot file.");
            if (size > PhysicsSnapshotMaxBytes) throw std::length_error("Physics snapshot file is too large.");
            std::ifstream input(path, std::ios::binary);
            if (!input) throw std::runtime_error("Failed to open physics snapshot file.");
            std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
            if (!bytes.empty()) input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            if (!input && !bytes.empty()) throw std::runtime_error("Failed while reading physics snapshot file.");
            return bytes;
        }
        inline void WriteFileRecoverably(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes)
        {
            const auto temp = std::filesystem::path(path.string() + ".tmp");
            const auto backup = std::filesystem::path(path.string() + ".bak");
            std::error_code ec; std::filesystem::remove(temp, ec); ec.clear();
            std::filesystem::remove(backup, ec); ec.clear();
            {
                std::ofstream output(temp, std::ios::binary | std::ios::trunc);
                if (!output) throw std::runtime_error("Failed to create temporary physics snapshot file.");
                if (!bytes.empty()) output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
                output.flush();
                if (!output) throw std::runtime_error("Failed while writing physics snapshot file.");
            }
            const bool hadTarget = std::filesystem::exists(path);
            if (hadTarget)
            {
                std::filesystem::rename(path, backup, ec);
                if (ec) { std::filesystem::remove(temp); throw std::runtime_error("Failed to stage previous physics snapshot file."); }
            }
            std::filesystem::rename(temp, path, ec);
            if (ec)
            {
                if (hadTarget) { std::error_code restore; std::filesystem::rename(backup, path, restore); }
                std::filesystem::remove(temp);
                throw std::runtime_error("Failed to publish physics snapshot file.");
            }
            if (hadTarget) { std::filesystem::remove(backup, ec); }
        }
    }

    [[nodiscard]] inline std::vector<std::uint8_t> SerializePhysicsWorldSnapshot(const PhysicsWorldSnapshot& snapshot)
    {
        using namespace serialization_detail;
        Writer w; w.Raw(SnapshotMagic); w.U32(PhysicsSnapshotFileVersion);
        WriteSettings(w, snapshot.Settings); w.Vec3(snapshot.Gravity); w.Float(snapshot.FixedAccumulator);
        w.Count(snapshot.Bodies.size()); for (const auto& body : snapshot.Bodies) WriteBody(w, body);
        w.Count(snapshot.Colliders.size()); for (const auto& collider : snapshot.Colliders) WriteCollider(w, collider);
        w.Count(snapshot.Joints.size()); for (const auto& joint : snapshot.Joints) WriteJoint(w, joint);
        w.Count(snapshot.LastPairs.size()); for (const auto& pair : snapshot.LastPairs) { w.U32(pair.A); w.U32(pair.B); }
        w.Count(snapshot.LastContacts.size()); for (const auto& m : snapshot.LastContacts) WriteManifold(w, m);
        w.Count(snapshot.PairResponses.size()); for (const auto& rule : snapshot.PairResponses) { w.U32(rule.ColliderA); w.U32(rule.ColliderB); WriteResponse(w, rule.Response); }
        w.Count(snapshot.LayerResponses.size()); for (const auto& rule : snapshot.LayerResponses) { w.U32(rule.LayerA); w.U32(rule.LayerB); WriteResponse(w, rule.Response); }
        w.Count(snapshot.PreviousContactKeys.size()); for (const auto& key : snapshot.PreviousContactKeys)
        { w.U32(key.BodyA); w.U32(key.BodyB); w.U32(key.ColliderA); w.U32(key.ColliderB); w.Bool(key.IsTrigger); WriteResponse(w, key.Response); }
        w.Count(snapshot.ContactCache.size()); for (const auto& e : snapshot.ContactCache)
        { w.U32(e.BodyA); w.U32(e.BodyB); w.U32(e.ColliderA); w.U32(e.ColliderB); w.Vec3(e.LocalAnchorA); w.Vec3(e.LocalAnchorB); w.Vec3(e.Normal); w.Float(e.NormalImpulse); w.Float(e.TangentImpulse); }
        return std::move(w.Bytes);
    }

    [[nodiscard]] inline PhysicsWorldSnapshot DeserializePhysicsWorldSnapshot(const std::vector<std::uint8_t>& bytes)
    {
        using namespace serialization_detail;
        if (bytes.size() > PhysicsSnapshotMaxBytes) throw std::length_error("Physics snapshot byte buffer is too large.");
        Reader r(bytes); r.Magic(SnapshotMagic);
        const auto fileVersion = r.U32();
        if (fileVersion != PhysicsSnapshotFileVersion) throw std::runtime_error("Physics snapshot file version is unsupported.");
        PhysicsWorldSnapshot s; s.Version = PhysicsWorldSnapshotVersion;
        s.Settings = ReadSettings(r); s.Gravity = r.Vec3(); s.FixedAccumulator = r.Float();
        auto count = r.Count(); s.Bodies.reserve(count); for (std::uint32_t i=0;i<count;++i) s.Bodies.push_back(ReadBody(r));
        count = r.Count(); s.Colliders.reserve(count); for (std::uint32_t i=0;i<count;++i) s.Colliders.push_back(ReadCollider(r));
        count = r.Count(); s.Joints.reserve(count); for (std::uint32_t i=0;i<count;++i) s.Joints.push_back(ReadJoint(r));
        count = r.Count(); s.LastPairs.reserve(count); for (std::uint32_t i=0;i<count;++i) s.LastPairs.push_back({ r.U32(), r.U32() });
        count = r.Count(); s.LastContacts.reserve(count); for (std::uint32_t i=0;i<count;++i) s.LastContacts.push_back(ReadManifold(r));
        count = r.Count(); s.PairResponses.reserve(count); for (std::uint32_t i=0;i<count;++i) s.PairResponses.push_back({ r.U32(), r.U32(), ReadResponse(r) });
        count = r.Count(); s.LayerResponses.reserve(count); for (std::uint32_t i=0;i<count;++i) s.LayerResponses.push_back({ r.U32(), r.U32(), ReadResponse(r) });
        count = r.Count(); s.PreviousContactKeys.reserve(count); for (std::uint32_t i=0;i<count;++i) s.PreviousContactKeys.push_back({ r.U32(), r.U32(), r.U32(), r.U32(), r.Bool(), ReadResponse(r) });
        count = r.Count(); s.ContactCache.reserve(count); for (std::uint32_t i=0;i<count;++i) s.ContactCache.push_back({ r.U32(), r.U32(), r.U32(), r.U32(), r.Vec3(), r.Vec3(), r.Vec3(), r.Float(), r.Float() });
        r.RequireEnd(); return s;
    }

    [[nodiscard]] inline std::uint64_t PhysicsStateHash(const PhysicsWorldSnapshot& snapshot)
    {
        const auto bytes = SerializePhysicsWorldSnapshot(snapshot);
        std::uint64_t hash = 14695981039346656037ull;
        for (const std::uint8_t byte : bytes) { hash ^= byte; hash *= 1099511628211ull; }
        return hash;
    }
    [[nodiscard]] inline std::uint64_t PhysicsStateHash(const PhysicsWorld& world)
    { return PhysicsStateHash(world.CaptureSnapshot()); }

    inline void SavePhysicsWorld(const std::filesystem::path& path, const PhysicsWorld& world)
    { serialization_detail::WriteFileRecoverably(path, SerializePhysicsWorldSnapshot(world.CaptureSnapshot())); }

    inline void LoadPhysicsWorld(const std::filesystem::path& path, PhysicsWorld& world)
    {
        const PhysicsWorldSnapshot snapshot =
            DeserializePhysicsWorldSnapshot(serialization_detail::ReadFile(path));
        PhysicsWorld validationTarget;
        validationTarget.RestoreSnapshot(snapshot);
        world.RestoreSnapshot(snapshot);
    }
}
