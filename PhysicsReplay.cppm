module;

#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Replay;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.World;
import Kairo.Foundation.PhysicsEngine.Serialization;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    inline constexpr std::uint32_t PhysicsReplayVersion = 1u;

    struct ReplayAddForce final { BodyID Body = InvalidBodyID; Vec3f Force = Vec3f::Zero(); };
    struct ReplayAddTorque final { BodyID Body = InvalidBodyID; Vec3f Torque = Vec3f::Zero(); };
    struct ReplayImpulseAtPoint final { BodyID Body = InvalidBodyID; Vec3f Impulse = Vec3f::Zero(); Vec3f Point = Vec3f::Zero(); };
    struct ReplaySetMotionState final { BodyID Body = InvalidBodyID; MotionState State; };
    struct ReplayWakeBody final { BodyID Body = InvalidBodyID; };
    struct ReplaySetCCD final { BodyID Body = InvalidBodyID; CollisionDetectionMode Mode = CollisionDetectionMode::Discrete; };

    using PhysicsReplayCommand = std::variant<ReplayAddForce, ReplayAddTorque,
        ReplayImpulseAtPoint, ReplaySetMotionState, ReplayWakeBody, ReplaySetCCD>;

    struct PhysicsReplayFrame final
    {
        std::vector<PhysicsReplayCommand> Commands;
        std::uint64_t ExpectedStateHash = 0u;
    };

    struct PhysicsReplay final
    {
        std::uint32_t Version = PhysicsReplayVersion;
        float FixedDt = 1.0f / 60.0f;
        PhysicsWorldSnapshot InitialState;
        std::vector<PhysicsReplayFrame> Frames;
    };

    struct PhysicsReplayVerification final
    {
        bool Matched = true;
        std::uint32_t VerifiedFrames = 0u;
        std::uint32_t DivergentFrame = std::numeric_limits<std::uint32_t>::max();
        std::uint64_t ExpectedHash = 0u;
        std::uint64_t ActualHash = 0u;
    };

    inline void ApplyPhysicsReplayCommand(PhysicsWorld& world, const PhysicsReplayCommand& command)
    {
        std::visit([&](const auto& value)
        {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, ReplayAddForce>)
                world.AddBodyForce(value.Body, value.Force);
            else if constexpr (std::is_same_v<T, ReplayAddTorque>)
                world.AddBodyTorque(value.Body, value.Torque);
            else if constexpr (std::is_same_v<T, ReplayImpulseAtPoint>)
                world.ApplyBodyImpulseAtPoint(value.Body, value.Impulse, value.Point);
            else if constexpr (std::is_same_v<T, ReplayWakeBody>)
                world.WakeBody(value.Body);
            else if constexpr (std::is_same_v<T, ReplaySetCCD>)
                world.SetBodyCollisionDetectionMode(value.Body, value.Mode);
            else if constexpr (std::is_same_v<T, ReplaySetMotionState>)
            {
                if (!world.IsValidBody(value.Body))
                    throw std::out_of_range("ReplaySetMotionState references an invalid body.");
                RequireFinite(value.State.Position, "ReplaySetMotionState.Position");
                RequireFinite(value.State.LinearVelocity, "ReplaySetMotionState.LinearVelocity");
                RequireFinite(value.State.AngularVelocity, "ReplaySetMotionState.AngularVelocity");
                RequireFinite(value.State.Rotation.x, "ReplaySetMotionState.Rotation");
                RequireFinite(value.State.Rotation.y, "ReplaySetMotionState.Rotation");
                RequireFinite(value.State.Rotation.z, "ReplaySetMotionState.Rotation");
                RequireFinite(value.State.Rotation.w, "ReplaySetMotionState.Rotation");
                world.Bodies().at(value.Body).State = value.State;
                if (IsDynamicBodyType(world.Bodies().at(value.Body))) world.WakeBody(value.Body);
            }
        }, command);
    }

    class PhysicsReplayRecorder final
    {
    public:
        PhysicsReplayRecorder(const PhysicsWorld& world, float fixedDt)
        {
            RequirePositive(fixedDt, "PhysicsReplayRecorder.fixedDt");
            m_Replay.FixedDt = fixedDt;
            m_Replay.InitialState = world.CaptureSnapshot();
        }

        void Step(PhysicsWorld& world, std::vector<PhysicsReplayCommand> commands = {})
        {
            for (const auto& command : commands) ApplyPhysicsReplayCommand(world, command);
            world.Step(m_Replay.FixedDt);
            m_Replay.Frames.push_back({ std::move(commands), PhysicsStateHash(world) });
        }

        [[nodiscard]] const PhysicsReplay& Replay() const noexcept { return m_Replay; }
        [[nodiscard]] PhysicsReplay TakeReplay() && { return std::move(m_Replay); }
    private:
        PhysicsReplay m_Replay;
    };

    [[nodiscard]] inline PhysicsReplayVerification VerifyPhysicsReplay(
        const PhysicsReplay& replay, PhysicsWorld& world)
    {
        if (replay.Version != PhysicsReplayVersion)
            throw std::invalid_argument("PhysicsReplay has an unsupported version.");
        RequirePositive(replay.FixedDt, "PhysicsReplay.FixedDt");
        world.RestoreSnapshot(replay.InitialState);
        PhysicsReplayVerification result;
        for (std::size_t index = 0u; index < replay.Frames.size(); ++index)
        {
            const PhysicsReplayFrame& frame = replay.Frames[index];
            for (const auto& command : frame.Commands) ApplyPhysicsReplayCommand(world, command);
            world.Step(replay.FixedDt);
            const std::uint64_t actual = PhysicsStateHash(world);
            if (actual != frame.ExpectedStateHash)
            {
                result.Matched = false;
                result.DivergentFrame = static_cast<std::uint32_t>(index);
                result.ExpectedHash = frame.ExpectedStateHash;
                result.ActualHash = actual;
                return result;
            }
            ++result.VerifiedFrames;
        }
        return result;
    }

    namespace replay_detail
    {
        inline constexpr std::array<std::uint8_t, 8> Magic{ 'K','P','R','E','P','0','1','\n' };
        inline constexpr std::size_t MaxBytes = 256u * 1024u * 1024u;
        inline constexpr std::uint32_t MaxFrames = 1'000'000u;
        inline constexpr std::uint32_t MaxCommandsPerFrame = 1'000'000u;

        struct Writer final
        {
            std::vector<std::uint8_t> b;
            void u8(std::uint8_t v){ b.push_back(v); }
            void u32(std::uint32_t v){ for(int s=0;s<32;s+=8)u8(static_cast<std::uint8_t>((v>>s)&0xffu)); }
            void u64(std::uint64_t v){ for(int s=0;s<64;s+=8)u8(static_cast<std::uint8_t>((v>>s)&0xffu)); }
            void f(float v){u32(std::bit_cast<std::uint32_t>(v));}
            void v3(const Vec3f& v){f(v.x);f(v.y);f(v.z);}
            void q(const Quaternionf& v){f(v.x);f(v.y);f(v.z);f(v.w);}
            void raw(const std::vector<std::uint8_t>& x)
            { if(x.size()>MaxBytes||b.size()>MaxBytes-x.size())throw std::length_error("Replay too large."); b.insert(b.end(),x.begin(),x.end()); }
            template<std::size_t N> void raw(const std::array<std::uint8_t,N>& x){b.insert(b.end(),x.begin(),x.end());}
            void count(std::size_t n,std::uint32_t max)
            {if(n>max||n>std::numeric_limits<std::uint32_t>::max())throw std::length_error("Replay count too large.");u32(static_cast<std::uint32_t>(n));}
        };
        struct Reader final
        {
            const std::vector<std::uint8_t>& b; std::size_t o=0;
            explicit Reader(const std::vector<std::uint8_t>& bytes):b(bytes){}
            void req(std::size_t n)const{if(o>b.size()||n>b.size()-o)throw std::runtime_error("Replay is truncated.");}
            std::uint8_t u8(){req(1);return b[o++];}
            std::uint32_t u32(){req(4);std::uint32_t v=0;for(int s=0;s<32;s+=8)v|=static_cast<std::uint32_t>(b[o++])<<s;return v;}
            std::uint64_t u64(){req(8);std::uint64_t v=0;for(int s=0;s<64;s+=8)v|=static_cast<std::uint64_t>(b[o++])<<s;return v;}
            float f(){return std::bit_cast<float>(u32());}
            Vec3f v3(){return{f(),f(),f()};} Quaternionf q(){return{f(),f(),f(),f()};}
            std::uint32_t count(std::uint32_t max){auto n=u32();if(n>max)throw std::runtime_error("Replay count exceeds limit.");return n;}
            std::vector<std::uint8_t> bytes(std::uint32_t n){req(n);auto first=b.begin()+static_cast<std::ptrdiff_t>(o);o+=n;return{first,first+n};}
            void magic(){for(auto x:Magic)if(u8()!=x)throw std::runtime_error("Replay magic is invalid.");}
            void end()const{if(o!=b.size())throw std::runtime_error("Replay contains trailing bytes.");}
        };
        inline void writeState(Writer& w,const MotionState& s){w.v3(s.Position);w.q(s.Rotation);w.v3(s.LinearVelocity);w.v3(s.AngularVelocity);}
        inline MotionState readState(Reader& r){return{r.v3(),r.q(),r.v3(),r.v3()};}
        inline void writeCommand(Writer& w,const PhysicsReplayCommand& c)
        {
            std::visit([&](const auto& x){using T=std::decay_t<decltype(x)>;
                if constexpr(std::is_same_v<T,ReplayAddForce>){w.u8(0);w.u32(x.Body);w.v3(x.Force);}
                else if constexpr(std::is_same_v<T,ReplayAddTorque>){w.u8(1);w.u32(x.Body);w.v3(x.Torque);}
                else if constexpr(std::is_same_v<T,ReplayImpulseAtPoint>){w.u8(2);w.u32(x.Body);w.v3(x.Impulse);w.v3(x.Point);}
                else if constexpr(std::is_same_v<T,ReplaySetMotionState>){w.u8(3);w.u32(x.Body);writeState(w,x.State);}
                else if constexpr(std::is_same_v<T,ReplayWakeBody>){w.u8(4);w.u32(x.Body);}
                else if constexpr(std::is_same_v<T,ReplaySetCCD>){w.u8(5);w.u32(x.Body);w.u8(x.Mode==CollisionDetectionMode::Continuous?1u:0u);}
            },c);
        }
        inline PhysicsReplayCommand readCommand(Reader& r)
        {
            switch(r.u8()){
            case 0:return ReplayAddForce{r.u32(),r.v3()};
            case 1:return ReplayAddTorque{r.u32(),r.v3()};
            case 2:return ReplayImpulseAtPoint{r.u32(),r.v3(),r.v3()};
            case 3:return ReplaySetMotionState{r.u32(),readState(r)};
            case 4:return ReplayWakeBody{r.u32()};
            case 5:{auto b=r.u32();auto m=r.u8();if(m>1)throw std::runtime_error("Replay CCD mode invalid.");return ReplaySetCCD{b,m?CollisionDetectionMode::Continuous:CollisionDetectionMode::Discrete};}
            default:throw std::runtime_error("Replay command tag is invalid.");}
        }
        inline std::vector<std::uint8_t> readFile(const std::filesystem::path& p)
        {std::error_code ec;auto n=std::filesystem::file_size(p,ec);if(ec)throw std::runtime_error("Failed to stat replay file.");if(n>MaxBytes)throw std::length_error("Replay file too large.");std::ifstream in(p,std::ios::binary);if(!in)throw std::runtime_error("Failed to open replay file.");std::vector<std::uint8_t>x(static_cast<std::size_t>(n));if(!x.empty())in.read(reinterpret_cast<char*>(x.data()),static_cast<std::streamsize>(x.size()));if(!in&&!x.empty())throw std::runtime_error("Failed while reading replay file.");return x;}
        inline void writeFile(const std::filesystem::path&p,const std::vector<std::uint8_t>&x)
        {auto t=std::filesystem::path(p.string()+".tmp");std::error_code ec;std::filesystem::remove(t,ec);std::ofstream out(t,std::ios::binary|std::ios::trunc);if(!out)throw std::runtime_error("Failed to create replay temp file.");if(!x.empty())out.write(reinterpret_cast<const char*>(x.data()),static_cast<std::streamsize>(x.size()));out.flush();if(!out)throw std::runtime_error("Failed while writing replay file.");out.close();auto bak=std::filesystem::path(p.string()+".bak");std::filesystem::remove(bak,ec);bool old=std::filesystem::exists(p);if(old){std::filesystem::rename(p,bak,ec);if(ec)throw std::runtime_error("Failed to stage old replay file.");}std::filesystem::rename(t,p,ec);if(ec){if(old){std::error_code xec;std::filesystem::rename(bak,p,xec);}throw std::runtime_error("Failed to publish replay file.");}if(old)std::filesystem::remove(bak,ec);}
    }

    [[nodiscard]] inline std::vector<std::uint8_t> SerializePhysicsReplay(const PhysicsReplay& replay)
    {
        using namespace replay_detail;
        if(replay.Version!=PhysicsReplayVersion)throw std::invalid_argument("Unsupported PhysicsReplay version.");
        RequirePositive(replay.FixedDt,"PhysicsReplay.FixedDt");
        Writer w;w.raw(Magic);w.u32(PhysicsReplayVersion);w.f(replay.FixedDt);
        const auto state=SerializePhysicsWorldSnapshot(replay.InitialState);w.count(state.size(),static_cast<std::uint32_t>(MaxBytes));w.raw(state);
        w.count(replay.Frames.size(),MaxFrames);
        for(const auto&frame:replay.Frames){w.count(frame.Commands.size(),MaxCommandsPerFrame);for(const auto&c:frame.Commands)writeCommand(w,c);w.u64(frame.ExpectedStateHash);}return std::move(w.b);
    }
    [[nodiscard]] inline PhysicsReplay DeserializePhysicsReplay(const std::vector<std::uint8_t>& bytes)
    {
        using namespace replay_detail;if(bytes.size()>MaxBytes)throw std::length_error("Replay buffer too large.");Reader r(bytes);r.magic();if(r.u32()!=PhysicsReplayVersion)throw std::runtime_error("Replay file version unsupported.");PhysicsReplay replay;replay.FixedDt=r.f();auto n=r.count(static_cast<std::uint32_t>(MaxBytes));replay.InitialState=DeserializePhysicsWorldSnapshot(r.bytes(n));auto frames=r.count(MaxFrames);replay.Frames.reserve(frames);for(std::uint32_t i=0;i<frames;++i){PhysicsReplayFrame f;auto commands=r.count(MaxCommandsPerFrame);f.Commands.reserve(commands);for(std::uint32_t j=0;j<commands;++j)f.Commands.push_back(readCommand(r));f.ExpectedStateHash=r.u64();replay.Frames.push_back(std::move(f));}r.end();return replay;
    }
    inline void SavePhysicsReplay(const std::filesystem::path& path,const PhysicsReplay& replay)
    {replay_detail::writeFile(path,SerializePhysicsReplay(replay));}
    [[nodiscard]] inline PhysicsReplay LoadPhysicsReplay(const std::filesystem::path& path)
    {return DeserializePhysicsReplay(replay_detail::readFile(path));}
}
