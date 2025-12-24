
#pragma once

#include <string>

namespace text {
inline std::string Bold(const std::string& str) { return "\033[1m" + str + "\033[0m"; }
//
inline std::string Green(const std::string& str) { return "\033[32m" + str + "\033[0m"; }
inline std::string Red(const std::string& str) { return "\033[31m" + str + "\033[0m"; }
inline std::string Yellow(const std::string& str) { return "\033[33m" + str + "\033[0m"; }
//
inline std::string GreenBG(const std::string& str) { return "\033[42m" + str + "\033[0m"; }
inline std::string RedBG(const std::string& str) { return "\033[41m" + str + "\033[0m"; }
inline std::string YellowBG(const std::string& str) { return "\033[43m" + str + "\033[0m"; }
}  // namespace text
