module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.VehicleSystem;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.World;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    using VehicleID = std::uint32_t;
    inline constexpr VehicleID InvalidVehicleID =
        std::numeric_limits<VehicleID>::max();

    /// One raycast wheel attached to a rigid chassis. The wheel itself is not a
    /// rigid body: suspension and tire forces are calculated from a ray query
    /// and applied to the chassis at the contact point. This is the same broad
    /// architecture used by real-time game vehicle solvers because it remains
    /// stable at high wheel speeds without requiring four tiny spinning rigid
    /// bodies and contact manifolds per car.
    struct VehicleWheelDesc final
    {
        Vec3f LocalMount = Vec3f::Zero();
        Vec3f LocalSuspensionDirection = Vec3f::Up() * -1.0f;
        Vec3f LocalForward = Vec3f::UnitZ();
        float Radius = 0.35f;
        float RestLength = 0.40f;
        float MaxCompression = 0.18f;
        float MaxDroop = 0.15f;
        float SpringStiffness = 32'000.0f;
        float CompressionDamping = 4'500.0f;
        float ReboundDamping = 5'500.0f;
        float LateralStiffness = 2'500.0f;
        float LongitudinalStiffness = 120.0f;
        float FrictionCoefficient = 1.15f;
        float DriveFactor = 0.0f;
        float BrakeFactor = 1.0f;
        float HandbrakeFactor = 0.0f;
        float SteerFactor = 0.0f;
        std::uint32_t GroundMask = 0xFFFF'FFFFu;

        void Validate() const
        {
            RequireFinite(LocalMount, "VehicleWheel.LocalMount");
            RequireFinite(LocalSuspensionDirection,
                "VehicleWheel.LocalSuspensionDirection");
            RequireFinite(LocalForward, "VehicleWheel.LocalForward");
            if (LocalSuspensionDirection.LengthSquared() <= 1.0e-8f ||
                LocalForward.LengthSquared() <= 1.0e-8f)
                throw std::invalid_argument(
                    "Vehicle wheel suspension and forward directions must be non-zero.");
            RequirePositive(Radius, "VehicleWheel.Radius");
            RequirePositive(RestLength, "VehicleWheel.RestLength");
            RequireNonNegative(MaxCompression, "VehicleWheel.MaxCompression");
            RequireNonNegative(MaxDroop, "VehicleWheel.MaxDroop");
            if (MaxCompression >= RestLength)
                throw std::invalid_argument(
                    "Vehicle wheel maximum compression must be smaller than rest length.");
            RequireNonNegative(SpringStiffness, "VehicleWheel.SpringStiffness");
            RequireNonNegative(CompressionDamping,
                "VehicleWheel.CompressionDamping");
            RequireNonNegative(ReboundDamping, "VehicleWheel.ReboundDamping");
            RequireNonNegative(LateralStiffness,
                "VehicleWheel.LateralStiffness");
            RequireNonNegative(LongitudinalStiffness,
                "VehicleWheel.LongitudinalStiffness");
            RequireNonNegative(FrictionCoefficient,
                "VehicleWheel.FrictionCoefficient");
            ValidateUnitFactor(DriveFactor, "VehicleWheel.DriveFactor");
            ValidateUnitFactor(BrakeFactor, "VehicleWheel.BrakeFactor");
            ValidateUnitFactor(HandbrakeFactor,
                "VehicleWheel.HandbrakeFactor");
            ValidateUnitFactor(SteerFactor, "VehicleWheel.SteerFactor");
            if (GroundMask == 0u)
                throw std::invalid_argument(
                    "Vehicle wheel ground mask cannot be empty.");
        }

    private:
        static void ValidateUnitFactor(float value, const char* name)
        {
            RequireFinite(value, name);
            if (value < 0.0f || value > 1.0f)
                throw std::invalid_argument(
                    "Vehicle wheel control factors must be in [0, 1].");
        }
    };

    /// Chassis-wide force limits. MaxDriveForce/MaxBrakeForce are totals for
    /// the vehicle and are distributed across wheels by their configured
    /// factors, so a four-wheel-drive car does not accidentally receive four
    /// times the authored engine force.
    struct VehicleDesc final
    {
        BodyID Chassis = InvalidBodyID;
        std::vector<VehicleWheelDesc> Wheels;
        float MaxSteerAngleRadians = 0.55f;
        float MaxDriveForce = 9'000.0f;
        float MaxBrakeForce = 14'000.0f;
        float MaxHandbrakeForce = 18'000.0f;
        float AerodynamicDrag = 0.30f;
        float DownforceCoefficient = 0.0f;
        float RollingResistance = 35.0f;

        void Validate() const
        {
            if (Chassis == InvalidBodyID)
                throw std::invalid_argument("Vehicle requires a valid chassis body.");
            if (Wheels.empty() || Wheels.size() > 32u)
                throw std::invalid_argument(
                    "Vehicle requires between one and thirty-two wheels.");
            RequireNonNegative(MaxSteerAngleRadians,
                "Vehicle.MaxSteerAngleRadians");
            if (MaxSteerAngleRadians > Pi * 0.5f)
                throw std::invalid_argument(
                    "Vehicle maximum steering angle cannot exceed ninety degrees.");
            RequireNonNegative(MaxDriveForce, "Vehicle.MaxDriveForce");
            RequireNonNegative(MaxBrakeForce, "Vehicle.MaxBrakeForce");
            RequireNonNegative(MaxHandbrakeForce,
                "Vehicle.MaxHandbrakeForce");
            RequireNonNegative(AerodynamicDrag, "Vehicle.AerodynamicDrag");
            RequireNonNegative(DownforceCoefficient,
                "Vehicle.DownforceCoefficient");
            RequireNonNegative(RollingResistance,
                "Vehicle.RollingResistance");
            for (const VehicleWheelDesc& wheel : Wheels) wheel.Validate();
        }
    };

    struct VehicleControls final
    {
        float Throttle = 0.0f;   // [-1, 1], negative is reverse/engine braking.
        float Steering = 0.0f;   // [-1, 1].
        float Brake = 0.0f;      // [0, 1].
        float Handbrake = 0.0f;  // [0, 1].

        void Validate() const
        {
            RequireFinite(Throttle, "VehicleControls.Throttle");
            RequireFinite(Steering, "VehicleControls.Steering");
            RequireFinite(Brake, "VehicleControls.Brake");
            RequireFinite(Handbrake, "VehicleControls.Handbrake");
            if (Throttle < -1.0f || Throttle > 1.0f ||
                Steering < -1.0f || Steering > 1.0f ||
                Brake < 0.0f || Brake > 1.0f ||
                Handbrake < 0.0f || Handbrake > 1.0f)
                throw std::invalid_argument(
                    "Vehicle controls are outside their normalized ranges.");
        }
    };

    struct VehicleWheelState final
    {
        bool Grounded = false;
        BodyID GroundBody = InvalidBodyID;
        ColliderID GroundCollider = InvalidColliderID;
        Vec3f ContactPoint = Vec3f::Zero();
        Vec3f ContactNormal = Vec3f::Up();
        float SuspensionLength = 0.0f;
        float SuspensionCompression = 0.0f;
        float SuspensionForce = 0.0f;
        float LongitudinalSpeed = 0.0f;
        float LateralSpeed = 0.0f;
        float LongitudinalForce = 0.0f;
        float LateralForce = 0.0f;
        float SteeringAngle = 0.0f;
        float AngularSpeed = 0.0f;
        float RotationAngle = 0.0f;
    };

    struct VehicleState final
    {
        std::size_t GroundedWheels = 0u;
        float Speed = 0.0f;
        float ForwardSpeed = 0.0f;
        Vec3f Forward = Vec3f::UnitZ();
        Vec3f Up = Vec3f::Up();
    };

    /// Deterministic raycast-wheel vehicle solver. Call Step before
    /// PhysicsWorld::Step for each fixed physics tick. The system accumulates
    /// suspension/tire/drag forces only; PhysicsWorld remains the sole owner of
    /// rigid-body integration, contacts, sleeping, serialization, and replay.
    class VehicleSystem final
    {
        struct VehicleRecord final
        {
            bool Active = true;
            VehicleDesc Desc;
            VehicleControls Controls;
            VehicleState State;
            std::vector<VehicleWheelState> WheelStates;
        };

    public:
        [[nodiscard]] VehicleID CreateVehicle(
            const PhysicsWorld& world, VehicleDesc desc)
        {
            desc.Validate();
            if (!world.IsValidBody(desc.Chassis))
                throw std::out_of_range(
                    "Vehicle chassis does not exist in the physics world.");
            const RigidBody& chassis = world.Bodies().at(desc.Chassis);
            if (!IsDynamicBodyType(chassis))
                throw std::invalid_argument(
                    "Vehicle chassis must be a finite-mass dynamic rigid body.");
            if (m_Vehicles.size() >=
                static_cast<std::size_t>(InvalidVehicleID))
                throw std::overflow_error("Vehicle ID space exhausted.");

            VehicleRecord record;
            record.Desc = std::move(desc);
            record.WheelStates.resize(record.Desc.Wheels.size());
            for (std::size_t index = 0u;
                index < record.Desc.Wheels.size(); ++index)
                record.WheelStates[index].SuspensionLength =
                    record.Desc.Wheels[index].RestLength +
                    record.Desc.Wheels[index].MaxDroop;
            const VehicleID id = static_cast<VehicleID>(m_Vehicles.size());
            m_Vehicles.push_back(std::move(record));
            return id;
        }

        void RemoveVehicle(VehicleID vehicle)
        {
            VehicleRecord& record = Require(vehicle);
            record.Active = false;
        }

        [[nodiscard]] bool IsValidVehicle(VehicleID vehicle) const noexcept
        {
            return vehicle < m_Vehicles.size() && m_Vehicles[vehicle].Active;
        }

        void SetControls(VehicleID vehicle, VehicleControls controls)
        {
            controls.Validate();
            Require(vehicle).Controls = controls;
        }

        [[nodiscard]] const VehicleControls& Controls(VehicleID vehicle) const
        {
            return Require(vehicle).Controls;
        }

        [[nodiscard]] const VehicleDesc& Descriptor(VehicleID vehicle) const
        {
            return Require(vehicle).Desc;
        }

        [[nodiscard]] const VehicleState& State(VehicleID vehicle) const
        {
            return Require(vehicle).State;
        }

        [[nodiscard]] const std::vector<VehicleWheelState>& Wheels(
            VehicleID vehicle) const
        {
            return Require(vehicle).WheelStates;
        }

        [[nodiscard]] std::size_t ActiveVehicleCount() const noexcept
        {
            return static_cast<std::size_t>(std::count_if(
                m_Vehicles.begin(), m_Vehicles.end(),
                [](const VehicleRecord& record) { return record.Active; }));
        }

        void Step(PhysicsWorld& world, float dt)
        {
            RequirePositive(dt, "VehicleSystem.dt");
            for (VehicleRecord& vehicle : m_Vehicles)
            {
                if (!vehicle.Active) continue;
                StepVehicle(world, vehicle, dt);
            }
        }

    private:
        std::vector<VehicleRecord> m_Vehicles;

        [[nodiscard]] VehicleRecord& Require(VehicleID id)
        {
            if (!IsValidVehicle(id))
                throw std::out_of_range("Vehicle id does not exist or is inactive.");
            return m_Vehicles[id];
        }

        [[nodiscard]] const VehicleRecord& Require(VehicleID id) const
        {
            if (!IsValidVehicle(id))
                throw std::out_of_range("Vehicle id does not exist or is inactive.");
            return m_Vehicles[id];
        }

        [[nodiscard]] static Vec3f RotateAroundAxis(
            const Vec3f& vector, const Vec3f& axis, float angle)
        {
            const Vec3f n = SafeNormalize(axis, Vec3f::Up());
            const float cosine = std::cos(angle);
            const float sine = std::sin(angle);
            return vector * cosine + Cross(n, vector) * sine +
                n * (Dot(n, vector) * (1.0f - cosine));
        }

        [[nodiscard]] static Vec3f ProjectToPlane(
            const Vec3f& vector, const Vec3f& normal,
            const Vec3f& fallback)
        {
            const Vec3f tangent = vector - normal * Dot(vector, normal);
            return SafeNormalize(tangent, fallback);
        }

        [[nodiscard]] static float FactorSum(
            const VehicleDesc& desc,
            float VehicleWheelDesc::* member)
        {
            float sum = 0.0f;
            for (const VehicleWheelDesc& wheel : desc.Wheels)
                sum += wheel.*member;
            return sum;
        }

        [[nodiscard]] static float DistributedForce(
            float total, float factor, float factorSum) noexcept
        {
            return factorSum > 1.0e-6f ? total * factor / factorSum : 0.0f;
        }

        [[nodiscard]] static float OpposingBrakeForce(
            float longitudinalSpeed, float authoredDrive,
            float brakeMagnitude) noexcept
        {
            if (brakeMagnitude <= 0.0f) return 0.0f;
            if (longitudinalSpeed > 0.05f) return -brakeMagnitude;
            if (longitudinalSpeed < -0.05f) return brakeMagnitude;
            // At a standstill, brakes cancel as much authored drive as they
            // can without creating motion in the opposite direction.
            return -std::clamp(authoredDrive,
                -brakeMagnitude, brakeMagnitude);
        }

        [[nodiscard]] static std::optional<PhysicsRayHit> GroundHit(
            const PhysicsWorld& world, BodyID chassis,
            const Vec3f& origin, const Vec3f& direction,
            float maxDistance, std::uint32_t mask)
        {
            for (const PhysicsRayHit& hit :
                world.RaycastAll(origin, direction, maxDistance, mask))
            {
                if (hit.Body == chassis || hit.IsTrigger) continue;
                return hit;
            }
            return std::nullopt;
        }

        static void ApplyPairForce(
            PhysicsWorld& world, BodyID chassis,
            const PhysicsRayHit& hit, const Vec3f& force,
            const Vec3f& chassisPoint)
        {
            world.AddBodyForceAtPoint(chassis, force, chassisPoint);
            if (world.IsValidBody(hit.Body) &&
                IsDynamicBodyType(world.Bodies().at(hit.Body)))
                world.AddBodyForceAtPoint(hit.Body, force * -1.0f, hit.Point);
        }

        static void StepVehicle(
            PhysicsWorld& world, VehicleRecord& vehicle, float dt)
        {
            const VehicleDesc& desc = vehicle.Desc;
            if (!world.IsValidBody(desc.Chassis))
                throw std::out_of_range(
                    "Vehicle chassis was removed from the physics world.");
            const RigidBody& initialBody = world.Bodies().at(desc.Chassis);
            if (!IsDynamicBodyType(initialBody))
                throw std::invalid_argument(
                    "Vehicle chassis is no longer a dynamic rigid body.");

            const Vec3f up = SafeNormalize(
                Rotate(initialBody.State.Rotation, Vec3f::Up()), Vec3f::Up());
            const Vec3f forward = SafeNormalize(
                Rotate(initialBody.State.Rotation, Vec3f::UnitZ()), Vec3f::UnitZ());
            vehicle.State.Up = up;
            vehicle.State.Forward = forward;
            vehicle.State.Speed = initialBody.State.LinearVelocity.Length();
            vehicle.State.ForwardSpeed = Dot(initialBody.State.LinearVelocity, forward);
            vehicle.State.GroundedWheels = 0u;

            const float speed = vehicle.State.Speed;
            if (speed > 1.0e-5f)
            {
                const Vec3f drag = initialBody.State.LinearVelocity *
                    (-desc.AerodynamicDrag * speed);
                world.AddBodyForce(desc.Chassis, drag);
            }
            if (desc.DownforceCoefficient > 0.0f && speed > 1.0e-5f)
                world.AddBodyForce(desc.Chassis,
                    up * (-desc.DownforceCoefficient * speed * speed));

            const float driveSum = FactorSum(desc, &VehicleWheelDesc::DriveFactor);
            const float brakeSum = FactorSum(desc, &VehicleWheelDesc::BrakeFactor);
            const float handbrakeSum =
                FactorSum(desc, &VehicleWheelDesc::HandbrakeFactor);

            for (std::size_t index = 0u; index < desc.Wheels.size(); ++index)
            {
                const VehicleWheelDesc& wheel = desc.Wheels[index];
                VehicleWheelState& state = vehicle.WheelStates[index];
                const float previousRotation = state.RotationAngle;
                state = VehicleWheelState{};
                state.RotationAngle = previousRotation;
                state.SteeringAngle = desc.MaxSteerAngleRadians *
                    vehicle.Controls.Steering * wheel.SteerFactor;
                state.SuspensionLength = wheel.RestLength + wheel.MaxDroop;

                // Re-read after earlier wheel forces only for consistent body
                // pose; forces accumulate but integration happens later in
                // PhysicsWorld::Step, so the pose is stable for all wheels.
                const RigidBody& body = world.Bodies().at(desc.Chassis);
                const Vec3f mount = body.State.Position +
                    Rotate(body.State.Rotation, wheel.LocalMount);
                const Vec3f suspensionDirection = SafeNormalize(
                    Rotate(body.State.Rotation,
                        wheel.LocalSuspensionDirection), Vec3f::Up() * -1.0f);
                const Vec3f suspensionUp = suspensionDirection * -1.0f;
                const float rayLength = wheel.RestLength + wheel.MaxDroop +
                    wheel.Radius;
                const auto hit = GroundHit(world, desc.Chassis, mount,
                    suspensionDirection, rayLength, wheel.GroundMask);
                if (!hit.has_value())
                {
                    state.AngularSpeed *= 0.98f;
                    state.RotationAngle = std::fmod(
                        state.RotationAngle + state.AngularSpeed * dt,
                        2.0f * Pi);
                    continue;
                }

                state.Grounded = true;
                ++vehicle.State.GroundedWheels;
                state.GroundBody = hit->Body;
                state.GroundCollider = hit->Collider;
                state.ContactPoint = hit->Point;
                state.ContactNormal = SafeNormalize(hit->Normal, suspensionUp);

                const float rawLength = hit->Distance - wheel.Radius;
                state.SuspensionLength = std::clamp(rawLength,
                    wheel.RestLength - wheel.MaxCompression,
                    wheel.RestLength + wheel.MaxDroop);
                state.SuspensionCompression =
                    wheel.RestLength - state.SuspensionLength;

                Vec3f groundVelocity = Vec3f::Zero();
                if (world.IsValidBody(hit->Body))
                    groundVelocity = VelocityAtPoint(
                        world.Bodies().at(hit->Body).State, hit->Point);
                const Vec3f chassisVelocity =
                    VelocityAtPoint(body.State, hit->Point);
                const Vec3f relativeVelocity =
                    chassisVelocity - groundVelocity;
                const float suspensionVelocity =
                    Dot(relativeVelocity, suspensionDirection);
                const float damping = suspensionVelocity >= 0.0f
                    ? wheel.CompressionDamping : wheel.ReboundDamping;
                state.SuspensionForce = std::max(0.0f,
                    state.SuspensionCompression * wheel.SpringStiffness +
                    suspensionVelocity * damping);
                if (state.SuspensionForce > 0.0f)
                    ApplyPairForce(world, desc.Chassis, *hit,
                        suspensionUp * state.SuspensionForce, mount);

                Vec3f wheelForward = SafeNormalize(
                    Rotate(body.State.Rotation, wheel.LocalForward), forward);
                wheelForward = RotateAroundAxis(wheelForward,
                    suspensionUp, state.SteeringAngle);
                const Vec3f contactForward = ProjectToPlane(
                    wheelForward, state.ContactNormal, forward);
                const Vec3f contactRight = SafeNormalize(
                    Cross(state.ContactNormal, contactForward),
                    ProjectToPlane(Rotate(body.State.Rotation, Vec3f::UnitX()),
                        state.ContactNormal, Vec3f::UnitX()));

                state.LongitudinalSpeed = Dot(relativeVelocity, contactForward);
                state.LateralSpeed = Dot(relativeVelocity, contactRight);
                state.AngularSpeed = state.LongitudinalSpeed / wheel.Radius;
                state.RotationAngle = std::fmod(
                    state.RotationAngle + state.AngularSpeed * dt,
                    2.0f * Pi);

                const float authoredDrive = DistributedForce(
                    desc.MaxDriveForce * vehicle.Controls.Throttle,
                    wheel.DriveFactor, driveSum);
                const float serviceBrake = DistributedForce(
                    desc.MaxBrakeForce * vehicle.Controls.Brake,
                    wheel.BrakeFactor, brakeSum);
                const float handbrake = DistributedForce(
                    desc.MaxHandbrakeForce * vehicle.Controls.Handbrake,
                    wheel.HandbrakeFactor, handbrakeSum);
                const float brakeForce = OpposingBrakeForce(
                    state.LongitudinalSpeed, authoredDrive,
                    serviceBrake + handbrake);
                const float rolling = -state.LongitudinalSpeed *
                    desc.RollingResistance;
                float longitudinal = authoredDrive + brakeForce + rolling -
                    state.LongitudinalSpeed * wheel.LongitudinalStiffness;
                float lateral = -state.LateralSpeed * wheel.LateralStiffness;

                // Tire force is constrained by a friction circle using the
                // current suspension load. This prevents independently-clamped
                // acceleration and cornering from exceeding available grip.
                const float tractionLimit =
                    wheel.FrictionCoefficient * state.SuspensionForce;
                const float requested = std::hypot(longitudinal, lateral);
                if (requested > tractionLimit && requested > 1.0e-6f)
                {
                    const float scale = tractionLimit / requested;
                    longitudinal *= scale;
                    lateral *= scale;
                }
                state.LongitudinalForce = longitudinal;
                state.LateralForce = lateral;
                const Vec3f tireForce = contactForward * longitudinal +
                    contactRight * lateral;
                if (tireForce.LengthSquared() > 1.0e-12f)
                    ApplyPairForce(world, desc.Chassis, *hit,
                        tireForce, hit->Point);
            }
        }
    };
}
