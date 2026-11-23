#pragma once
#include <string>
#include <utility>
#include <vector>
#include "g1_text_table.h"

namespace g1_keys {
enum class Input { Keyboard, UsbGamepad, UnitreeRemote };
enum class Motion { Holonomic, Longitudinal };
enum class Action { Handover, Reset, Policy, ZeroVelocity, Stop, Move, Turn };

struct Entry {
    std::string key;
    std::string text;
};

struct Layout {
    Input input;
    Motion motion;

    Entry Lookup(Action action) const {
        const bool keyboard_only = input == Input::Keyboard;
        const bool hardware = input == Input::UnitreeRemote;
        const bool lateral = motion == Motion::Holonomic;
        switch (action) {
            case Action::Handover:
                return {keyboard_only ? "9" : "9 / Start",
                        hardware ? "enable hardware output" : "enable simulator output"};
            case Action::Reset:  return {keyboard_only ? "p" : "p / X", "reset to default pose"};
            case Action::Policy: return {keyboard_only ? "r" : "r / A", "run policy"};
            case Action::Stop:
                return {hardware ? "Space / B / Select" : keyboard_only ? "Space" : "Space / B",
                        "stop and exit"};
            case Action::ZeroVelocity:
                return {keyboard_only ? "x" : "x / Y", "zero velocity"};
            case Action::Move:
                return {keyboard_only ? (lateral ? "w/s/a/d" : "w/s") :
                                        (lateral ? "Left stick Y/X" : "Left stick Y"),
                        lateral ? "move forward, back or sideways" : "adjust longitudinal speed"};
            case Action::Turn: return {keyboard_only ? "q/e" : "Right stick X", "turn"};
        }
        return {};
    }

    const char* InputName() const {
        switch (input) {
            case Input::Keyboard:      return "keyboard";
            case Input::UsbGamepad:    return "keyboard + USB gamepad";
            case Input::UnitreeRemote: return "keyboard + Unitree remote";
        }
        return "";
    }

    template <typename... Actions>
    std::string AvailableKeys(const std::string& state, Actions... actions) const {
        const Entry entries[] = {Lookup(actions)...};
        std::vector<std::pair<std::string, std::string>> rows;
        rows.reserve(sizeof...(Actions));
        for (const auto& entry : entries) rows.emplace_back(entry.key, entry.text);
        return "Available keys (" + state + ", " + InputName() + "):\n" + g1_text::KeyTable(rows);
    }
};

inline std::string Usage(Motion motion) {
    std::string usage;
    for (const auto input : {Input::Keyboard, Input::UsbGamepad, Input::UnitreeRemote}) {
        const Layout keys{input, motion};
        usage += "  " + std::string(keys.InputName()) + ": " + keys.Lookup(Action::Handover).key +
                 " to enable output, " + keys.Lookup(Action::Reset).key + " to reset, " +
                 keys.Lookup(Action::Policy).key + " to run policy, " +
                 keys.Lookup(Action::ZeroVelocity).key + " to zero velocity, " +
                 keys.Lookup(Action::Stop).key + " to stop and exit.\n";
    }
    return usage;
}
}  // namespace g1_keys
