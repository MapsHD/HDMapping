#pragma once

#include <filesystem>
#include <spdlog/fmt/fmt.h>

template<>
struct fmt::formatter<std::filesystem::path> : fmt::formatter<std::string>
{
    template<typename FormatContext>
    auto format(const std::filesystem::path& p, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(p.string(), ctx);
    }
};