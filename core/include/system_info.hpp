#pragma once

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <intrin.h>
#include <windows.h>
#else
#include <fstream>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#endif

namespace system_info
{

    // Get OS name with version
    inline std::string getOSName()
    {
#ifdef _WIN32
        // Read Windows build number from registry
        HKEY hKey;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion", 0, KEY_READ, &hKey) == ERROR_SUCCESS)
        {
            char buildStr[32] = { 0 };
            DWORD size = sizeof(buildStr);
            if (RegQueryValueExA(hKey, "CurrentBuildNumber", nullptr, nullptr, reinterpret_cast<LPBYTE>(buildStr), &size) == ERROR_SUCCESS)
            {
                RegCloseKey(hKey);
                int build = std::atoi(buildStr);
                // Windows 11: Build >= 22000, Windows 10: Build < 22000
                if (build >= 22000)
                    return "Windows 11";
                else if (build >= 10240)
                    return "Windows 10";
                else if (build >= 9600)
                    return "Windows 8.1";
                else if (build >= 9200)
                    return "Windows 8";
                else if (build >= 7600)
                    return "Windows 7";
            }
            RegCloseKey(hKey);
        }
        return "Windows";
#else
        // Try to read distribution info from /etc/os-release
        std::ifstream osRelease("/etc/os-release");
        std::string line;
        std::string prettyName;
        while (std::getline(osRelease, line))
        {
            if (line.find("PRETTY_NAME=") == 0)
            {
                prettyName = line.substr(12);
                // Remove quotes
                if (!prettyName.empty() && prettyName.front() == '"')
                    prettyName = prettyName.substr(1);
                if (!prettyName.empty() && prettyName.back() == '"')
                    prettyName.pop_back();
                return prettyName;
            }
        }
        // Fallback to kernel info
        struct utsname buf;
        if (uname(&buf) == 0)
            return std::string(buf.sysname) + " " + buf.release;
        return "Linux";
#endif
    }

    // Get CPU name
    inline std::string getCPUName()
    {
#ifdef _WIN32
        int cpuInfo[4] = { 0 };
        char brand[49] = { 0 };
        __cpuid(cpuInfo, 0x80000002);
        memcpy(brand, cpuInfo, sizeof(cpuInfo));
        __cpuid(cpuInfo, 0x80000003);
        memcpy(brand + 16, cpuInfo, sizeof(cpuInfo));
        __cpuid(cpuInfo, 0x80000004);
        memcpy(brand + 32, cpuInfo, sizeof(cpuInfo));
        // Trim leading spaces
        std::string result(brand);
        size_t start = result.find_first_not_of(' ');
        if (start != std::string::npos)
            result = result.substr(start);
        return result;
#else
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuinfo, line))
        {
            if (line.find("model name") != std::string::npos)
            {
                auto pos = line.find(':');
                if (pos != std::string::npos)
                    return line.substr(pos + 2);
            }
        }
        return "Unknown";
#endif
    }

    // Round up to nearest common RAM size (including mixed configurations like 32+16=48)
    inline int roundToMarketedRAM(double gibRam)
    {
        // Common RAM sizes in GB (powers of 2 + common mixed configs)
        const int sizes[] = { 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512 };
        for (int size : sizes)
        {
            // If within ~10% below the marketed size, it's likely that size
            if (gibRam <= size && gibRam >= size * 0.9)
                return size;
        }
        // Fallback: round to nearest integer
        return static_cast<int>(std::round(gibRam));
    }

    // Get total RAM in GB
    inline std::string getTotalRAM()
    {
#ifdef _WIN32
        MEMORYSTATUSEX memInfo;
        memInfo.dwLength = sizeof(MEMORYSTATUSEX);
        GlobalMemoryStatusEx(&memInfo);
        // Convert bytes to GB (1 GB = 10^9 bytes, matches marketed RAM on Windows)
        int gbRam = static_cast<int>(std::round(memInfo.ullTotalPhys / 1e9));
        return std::to_string(gbRam) + " GB";
#else
        struct sysinfo info;
        if (sysinfo(&info) == 0)
        {
            // Convert bytes to GiB (1 GiB = 1024^3 bytes)
            double gibRam = (static_cast<double>(info.totalram) * info.mem_unit) / (1024.0 * 1024.0 * 1024.0);
            // Round to marketed RAM size
            int gbRam = roundToMarketedRAM(gibRam);
            return std::to_string(gbRam) + " GB";
        }
        return "Unknown";
#endif
    }

} // namespace system_info
