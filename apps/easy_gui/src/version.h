#pragma once

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <string>

namespace easy_gui
{
    //! Easy Forest HDMapping's own release version (EASY_FOREST_VERSION in apps/easy_gui/CMakeLists.txt, matching the
    //! easy-forest-v* release tags). HDMapping's version is a different number, so user-facing text names both.
    inline constexpr const char* kEasyForestVersion = EASY_FOREST_VERSION;

    //! "Easy Forest HDMapping 0.1.0 (HDMapping 0.105.0)": window title, README.txt, easy_gui.log.
    inline std::string versionString()
    {
        return std::string("Easy Forest HDMapping ") + kEasyForestVersion + " (HDMapping " + HDMAPPING_CONFIGURE_PROJECT_VERSION + ")";
    }
} // namespace easy_gui
