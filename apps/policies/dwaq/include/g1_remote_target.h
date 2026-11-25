#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace g1_dwaq_app {
enum class RemoteMode { Velocity, Joints };

struct RemoteTarget {
    std::array<float, 3> velocity{};
    std::array<float, 29> position{}, joint_velocity{};
};

class RemoteInput {
public:
    RemoteInput(const std::string& path, RemoteMode mode,
                const std::array<float, 29>& lower, const std::array<float, 29>& upper)
        : path_(path), mode_(mode), lower_(lower), upper_(upper) {
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (path.empty() || path.front() != '/' || path.size() >= sizeof(address.sun_path))
            throw std::invalid_argument("Remote socket requires a short absolute path");
        const auto parent = path.substr(0, path.find_last_of('/'));
        struct stat info{};
        if (stat(parent.c_str(), &info) != 0 || !S_ISDIR(info.st_mode) ||
            info.st_uid != geteuid() || (info.st_mode & 0777) != 0700)
            throw std::invalid_argument("Put the remote socket in a private directory (mktemp -d)");
        std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
        fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw std::runtime_error("Cannot create remote velocity socket");
        if (bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            close(fd_);
            throw std::runtime_error("Cannot bind remote velocity socket: " + path);
        }
    }

    RemoteInput(const RemoteInput&) = delete;
    RemoteInput& operator=(const RemoteInput&) = delete;
    ~RemoteInput() {
        close(fd_);
        unlink(path_.c_str());
    }

    bool Arm() {
        if (armed_) return false;
        armed_ = true;
        armed_at_ = last_capture_ = Now();
        session_ = sequence_ = 0;
        return true;
    }

    void Lock() {
        armed_ = false;
        session_ = 0;
    }

    const RemoteTarget* Poll(const char*& fault) {
        fault = armed_ && Now() - last_capture_ >= kTimeoutNs ? "no fresh remote target" : nullptr;
        for (int packet = 0; !fault && packet < 8; ++packet) {
            std::array<unsigned char, 24 + 58 * 4> payload{};
            const auto length = recv(fd_, payload.data(), payload.size(), MSG_TRUNC);
            if (length < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && armed_)
                    fault = "local socket read failed";
                break;
            }
            if (!armed_) continue;
            if (length >= 24 && !session_ && Decode(payload.data() + 16, 8) < armed_at_) continue;
            if (length == 24 && !Decode(payload.data(), 8)) {
                fault = "bridge disconnected or rejected target";
                break;
            }
            if (length != (mode_ == RemoteMode::Joints ? 256 : 36)) {
                fault = "invalid local packet";
                break;
            }
            const auto session = Decode(payload.data(), 8);
            const auto sequence = Decode(payload.data() + 8, 8);
            const auto captured_at = Decode(payload.data() + 16, 8);
            const auto now = Now();
            if (!session) {
                fault = "bridge disconnected or rejected target";
                break;
            }
            if (captured_at > now || now - captured_at >= kTimeoutNs) {
                fault = "expired observation";
                break;
            }
            if (!sequence || (session_ && (session != session_ || sequence <= sequence_ ||
                                          captured_at <= last_capture_))) {
                fault = "wrong session or out-of-order target";
                break;
            }
            if (mode_ == RemoteMode::Velocity) {
                const float forward = ReadFloat(payload.data() + 24);
                const float lateral = ReadFloat(payload.data() + 28);
                const float yaw = ReadFloat(payload.data() + 32);
                if (!std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(yaw)) {
                    fault = "non-finite velocity";
                    break;
                }
                target_.velocity = {std::clamp(forward, -0.10f, 0.10f), 0.0f,
                                    std::clamp(yaw, -0.15f, 0.15f)};
            } else {
                for (std::size_t joint = 0; joint < 29; ++joint) {
                    const float position = ReadFloat(payload.data() + 24 + joint * 4);
                    const float speed = ReadFloat(payload.data() + 24 + (29 + joint) * 4);
                    if (!std::isfinite(position) || position < lower_[joint] || position > upper_[joint] ||
                        !std::isfinite(speed) || std::abs(speed) > 1.0f) {
                        fault = "whole-body reference outside local limits";
                        break;
                    }
                    target_.position[joint] = position;
                    target_.joint_velocity[joint] = speed;
                }
                if (fault) break;
            }
            session_ = session;
            sequence_ = sequence;
            last_capture_ = captured_at;
        }
        if (!fault && armed_ && Now() - last_capture_ >= kTimeoutNs)
            fault = "no fresh remote target";
        if (fault) Lock();
        return armed_ && session_ ? &target_ : nullptr;
    }

private:
    static constexpr uint64_t kTimeoutNs = 250'000'000;

    static uint64_t Now() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static uint64_t Decode(const unsigned char* bytes, std::size_t size) {
        uint64_t value = 0;
        for (std::size_t index = 0; index < size; ++index)
            value |= uint64_t(bytes[index]) << (index * 8);
        return value;
    }

    static float ReadFloat(const unsigned char* bytes) {
        const auto bits = static_cast<uint32_t>(Decode(bytes, 4));
        float value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::string path_;
    RemoteMode mode_;
    std::array<float, 29> lower_, upper_;
    int fd_;
    bool armed_ = false;
    uint64_t armed_at_ = 0, last_capture_ = 0, session_ = 0, sequence_ = 0;
    RemoteTarget target_;
};
}  // namespace g1_dwaq_app
