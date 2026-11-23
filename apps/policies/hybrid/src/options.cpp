#include "g1_hybrid_options.h"
#include "g1_key_dictionary.h"
#include <iostream>

namespace g1_hybrid_app {
Options ParseOptions(int argc, char** argv) { return g1_parkour_app::ParseOptions(argc, argv, "hybrid"); }
void PrintUsage(const std::string& program) {
    std::cout << program << " --sim [--domain 1 --interface lo --joystick DEVICE]\n"
              << program << " --hardware --domain 0 --interface ROBOT_IF\n"
              << "Common: --config JSON --check-policy\n"
              << "DWaQ for stand/backward/turn; sustained forward command selects Parkour.\n"
              << "Switch thresholds and both policy configurations come from JSON.\n"
              << "Startup is dry-run. Hardware uses the Unitree remote.\n"
              << g1_keys::Usage(g1_keys::Motion::Longitudinal);
}
}  // namespace g1_hybrid_app
