#pragma once

#include "g1_config_utils.h"
#include "g1_control_options.h"
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace g1_control {
// 解析 --profile 任务档案（profiles/<name>.json 或 JSON 路径）：
// 1) program 守卫：档案目标程序必须与当前二进制一致；
// 2) policy_config 覆盖 options.config（相对路径按档案文件解析）；
// 3) camera.config 解析到 options.camera_config；hardware + 需要深度的程序 +
//    pipeline "none"（原生流）互斥——D435 单深度传感器不能同时喂策略管线与原生流。
#ifndef G1_PROFILES_DIR
#define G1_PROFILES_DIR "profiles"
#endif
inline void ApplyProfile(RuntimeOptions& options, const std::string& program, bool depth_required) {
    if (options.profile.empty()) return;
    std::filesystem::path path(options.profile);
    const bool profile_name = !path.is_absolute() && !path.has_parent_path() && path.extension().empty();
    if (!path.is_absolute() && (profile_name || !std::filesystem::exists(path))) {
        const char* profiles_env = std::getenv("G1_PROFILES_DIR");
        const auto profiles = std::filesystem::path(profiles_env ? profiles_env : G1_PROFILES_DIR);
        path = profiles / (options.profile + ".json");
    }
    if (!std::filesystem::exists(path)) throw std::invalid_argument("Profile not found: " + options.profile);
    const auto root = YAML::LoadFile(path.string());
    if (!root["program"]) throw std::invalid_argument("Profile lacks a program field: " + path.string());
    const std::string profile_program = root["program"].as<std::string>();
    if (profile_program != program)
        throw std::invalid_argument("Profile '" + options.profile + "' targets program '" + profile_program +
                                    "', not '" + program + "'");
    if (root["policy_config"])
        options.config = g1_config::ResolvePath(path.string(), root["policy_config"].as<std::string>());
    if (root["camera"] && root["camera"]["config"])
        options.camera_config = g1_config::ResolvePath(path.string(), root["camera"]["config"].as<std::string>());
    // pipeline 以相机模块文件为单一事实来源；档案层字段仅作后备。
    std::string pipeline;
    if (root["camera"] && root["camera"]["pipeline"])
        pipeline = root["camera"]["pipeline"].as<std::string>();
    if (pipeline.empty() && !options.camera_config.empty() && std::filesystem::exists(options.camera_config)) {
        const auto camera_root = YAML::LoadFile(options.camera_config);
        if (camera_root["pipeline"]) pipeline = camera_root["pipeline"].as<std::string>();
    }
    if (options.hardware && depth_required && pipeline == "none")
        throw std::invalid_argument(
            "Profile '" + options.profile + "' selects the raw depth stream, which cannot coexist with the "
            "processed depth that '" + program + "' requires on hardware (single depth sensor). "
            "Run it in simulation, use a program without a depth policy, or pick a camera profile with the program pipeline.");
}
}  // namespace g1_control
