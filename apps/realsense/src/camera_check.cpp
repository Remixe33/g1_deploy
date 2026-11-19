#include "g1_depth_camera.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
// 从相机配置 JSON 读取 stream 段；缺省沿用 parkour 管线的 480x270@60。
g1_camera::Config LoadStream(const std::string& path) {
    g1_camera::Config config;
    if (path.empty()) return config;
    const auto root = YAML::LoadFile(path);
    const auto stream = root["stream"];
    if (!stream) throw std::runtime_error("Camera config lacks a stream section: " + path);
    config.width = stream["width"].as<int>();
    config.height = stream["height"].as<int>();
    config.fps = stream["fps"].as<int>();
    if (config.width <= 0 || config.height <= 0 || config.fps <= 0)
        throw std::runtime_error("Camera stream values must be positive: " + path);
    return config;
}
}  // namespace

int main(int argc, char** argv) {
    std::string camera_config;
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "g1_camera_check: capture 60 fresh USB depth frames using C++; no DDS or robot commands\n"
                  << "Optional: --camera-config JSON (stream section; default 480x270@60)\n";
        return 0;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--camera-config" && index + 1 < argc) camera_config = argv[++index];
        else { std::cerr << "Use g1_camera_check [--camera-config JSON]\n"; return 2; }
    }
    try {
        const auto config = LoadStream(camera_config);
        g1_camera::RealSenseCamera camera(config);
        const auto deadline = g1_camera::Clock::now() + std::chrono::seconds(5);
        uint64_t last_sequence = 0;
        int count = 0;
        const auto expected = static_cast<std::size_t>(config.width) * config.height;
        while (count < 60) {
            if (g1_camera::Clock::now() >= deadline) throw std::runtime_error("Camera check timed out");
            if (const auto sample = camera.Latest()) {
                const double now = std::chrono::duration<double>(g1_camera::Clock::now().time_since_epoch()).count();
                const auto& frame = sample->frame;
                if (frame.sequence <= last_sequence || now - frame.captured_at < -.05 || now - frame.captured_at >= .15 ||
                    !frame.raw_depth || frame.raw_depth->size() != expected ||
                    frame.width != config.width || frame.height != config.height) {
                    throw std::runtime_error("Camera check received stale or invalid depth");
                }
                last_sequence = frame.sequence;
                ++count;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "C++ RealSense: 60 fresh " << config.width << "x" << config.height << " depth frames verified at "
                  << config.fps << " Hz target; no robot commands\n";
    } catch (const std::exception& error) {
        std::cerr << "Camera check: " << error.what() << '\n';
        return 1;
    }
}
