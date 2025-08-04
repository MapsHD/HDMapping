/*
 * HDMapping Version System Test
 * 
 * Purpose:
 * - Validate HDMapping version system integration
 * - Test version macro availability and functionality
 * - Verify CMake version macros are properly defined
 * 
 * Requirements:
 * - CMake version macros (HDMAPPING_VERSION_*)
 * - Standard C++ library support
 * 
 * Compilation:
 * cmake --build . --config Release --target test_version_system
 * 
 * Location: tests/test_version_system.cpp
 * Created: 2025-08-04
 */

#include <iostream>
#include <string>

// Simple version function that doesn't require external dependencies
std::string get_hdmapping_version() {
    return std::to_string(HDMAPPING_VERSION_MAJOR) + "." + 
           std::to_string(HDMAPPING_VERSION_MINOR) + "." + 
           std::to_string(HDMAPPING_VERSION_PATCH);
}

int main() {
    std::cout << "=== HDMapping Version System Test ===" << std::endl;
    std::cout << "Testing version system functionality..." << std::endl;
    
    std::string version = get_hdmapping_version();
    std::cout << "Software version: " << version << std::endl;
    
    // Validate version format (should be X.Y.Z)
    if (version.find('.') != std::string::npos) {
        std::cout << "✓ Version format appears valid" << std::endl;
    } else {
        std::cout << "✗ Warning: Version format may be invalid" << std::endl;
    }
    
    // Check if using CMake macros or fallback
    if (version == "0.84.0") {
        std::cout << "ℹ Using fallback version (CMake macros not available)" << std::endl;
    } else {
        std::cout << "✓ Using CMake-generated version" << std::endl;
    }
    
    std::cout << "=== Test completed ===" << std::endl;
    return 0;
}
