#pragma once
#include <string>
#include <utility>
#include <vector>
#include "g1_text_table.h"

namespace g1_keys {
// 按键→说明词典（单一事实来源）：控制器状态清单按动作名引用条目，
// 键位写法与说明只在这里维护，log 经 KeyTable 渲染成表格。
enum class Action { Handover, Reset, Policy, ZeroVelocity, Stop, Move, Turn, MoveForward };

struct Entry {
    std::string key;
    std::string text;
};

inline Entry Lookup(Action action) {
    switch (action) {
        case Action::Handover:     return {"9 / Start", "real handover"};
        case Action::Reset:        return {"P / X", "reset to default pose"};
        case Action::Policy:       return {"R / A", "run policy"};
        case Action::ZeroVelocity: return {"X / Y", "zero velocity"};
        case Action::Stop:         return {"Space / B", "stop/exit"};
        case Action::Move:         return {"WASD / left stick", "move (forward, back, lateral)"};
        case Action::Turn:         return {"QE / right stick", "turn"};
        case Action::MoveForward:  return {"Left stick Y", "forward only (0..1)"};
    }
    return {};
}

// 按动作名列表渲染键位表格：KeyTable(Action::Reset, Action::Stop)。
template <typename... Actions>
std::string KeyTable(Actions... actions) {
    const Entry entries[] = {Lookup(actions)...};
    std::vector<std::pair<std::string, std::string>> rows;
    rows.reserve(sizeof...(Actions));
    for (const auto& entry : entries) rows.emplace_back(entry.key, entry.text);
    return g1_text::KeyTable(rows);
}

// "Available keys (状态)" 标题 + 表格的组合，调用点只写状态名与动作序列。
template <typename... Actions>
std::string AvailableKeys(const std::string& state, Actions... actions) {
    return "Available keys (" + state + "):\n" + KeyTable(actions...);
}
}  // namespace g1_keys
