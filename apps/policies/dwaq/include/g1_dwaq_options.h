#pragma once

#include <string>
#include "g1_control_options.h"
#include "g1_remote_target.h"

namespace g1_dwaq_app {
enum class Mode { Control, ReadOnly, CheckPolicy, CheckJoystick, Help };

struct Options : g1_control::RuntimeOptions {
    Mode mode = Mode::Control;
    std::string program;
    int help_exit_code = 0;
    std::string joystick;
    std::string remote_socket;
    RemoteMode remote_mode = RemoteMode::Velocity;
    std::string fixed_joints;
};

// 实现在 src/options.cpp：解析启动参数并检查组合是否合法。
Options ParseOptions(int argc, char** argv);
void PrintUsage(const std::string& program);
}  // namespace g1_dwaq_app
