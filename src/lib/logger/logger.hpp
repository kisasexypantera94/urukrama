#pragma once

#include <spdlog/spdlog.h>

namespace ksp::log {

inline void Debug(const auto& msg) { spdlog::debug(msg); }

template <typename... Args>
inline void Debug(spdlog::format_string_t<Args...> format, Args&&... args)
{
    spdlog::debug(format, std::forward<Args>(args)...);
}

inline void Info(const auto& msg) { spdlog::info(msg); }

template <typename... Args>
inline void Info(spdlog::format_string_t<Args...> format, Args&&... args)
{
    spdlog::info(format, std::forward<Args>(args)...);
}

inline void Warn(const auto& msg) { spdlog::warn(msg); }

template <typename... Args>
inline void Warn(spdlog::format_string_t<Args...> format, Args&&... args)
{
    spdlog::warn(format, std::forward<Args>(args)...);
}

inline void Error(const auto& msg) { spdlog::error(msg); }

template <typename... Args>
inline void Error(spdlog::format_string_t<Args...> format, Args&&... args)
{
    spdlog::error(format, std::forward<Args>(args)...);
}

inline void Critical(const auto& msg) { spdlog::critical(msg); }

template <typename... Args>
inline void Critical(spdlog::format_string_t<Args...> format, Args&&... args)
{
    spdlog::critical(format, std::forward<Args>(args)...);
}

}  // namespace ksp::log