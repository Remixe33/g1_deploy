#pragma once
#include <algorithm>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace g1_text {
inline std::string Repeat(const std::string& glyph, std::size_t count) {
    std::string out;
    out.reserve(glyph.size() * count);
    for (std::size_t index = 0; index < count; ++index) out += glyph;
    return out;
}
// 两列键位表格（内容为 ASCII），按最长行自动对齐，返回多行字符串（末尾带换行）。
template <typename Rows>
std::string KeyTable(const Rows& rows) {
    std::size_t keys = 0, texts = 0;
    for (const auto& row : rows) {
        keys = std::max(keys, std::string(row.first).size());
        texts = std::max(texts, std::string(row.second).size());
    }
    const std::string key_bar = Repeat("─", keys + 2), text_bar = Repeat("─", texts + 2);
    const auto row_line = [&](const std::string& key, const std::string& text) {
        return "│ " + key + std::string(keys - key.size(), ' ') + " │ " + text +
               std::string(texts - text.size(), ' ') + " │\n";
    };
    std::string out = "┌" + key_bar + "┬" + text_bar + "┐\n";
    out += row_line("Key", "Action");
    out += "├" + key_bar + "┼" + text_bar + "┤\n";
    for (const auto& row : rows) out += row_line(row.first, row.second);
    out += "└" + key_bar + "┴" + text_bar + "┘\n";
    return out;
}
inline std::string KeyTable(std::initializer_list<std::pair<const char*, const char*>> rows) {
    return KeyTable(std::vector<std::pair<std::string, std::string>>(
        rows.begin(), rows.end()));
}
}  // namespace g1_text
