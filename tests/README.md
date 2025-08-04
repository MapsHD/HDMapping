# HDMapping Tests

This directory contains test files and utilities for HDMapping development and validation.

## Test Files

### `test_version_system.cpp`
- **Purpose:** Tests the HDMapping version system and CMake macro integration
- **Usage:** Validates that `get_software_version()` function works correctly
- **Compile:** Requires linking to lidar_odometry_utils.h
- **Dependencies:** CMake version macros (HDMAPPING_VERSION_MAJOR, etc.)

### `test_toml_structure.toml`
- **Purpose:** Template and test file for TOML configuration structure
- **Usage:** Demonstrates correct `version_info` section format
- **Testing:** Used with `CheckConfigVersion()` function
- **Template:** Reference for creating new TOML configuration files

## Building Tests

### Version System Test
```bash
# From HDMapping root directory
cd tests
g++ -I../apps/lidar_odometry_step_1 \
    -I../shared/include \
    -DHDMAPPING_VERSION_MAJOR=0 \
    -DHDMAPPING_VERSION_MINOR=85 \
    -DHDMAPPING_VERSION_PATCH=0 \
    test_version_system.cpp -o test_version_system

# Run test
./test_version_system
```

### TOML Structure Test
```bash
# Test TOML parsing with HDMapping applications
cd ../apps/lidar_odometry_step_1
./enhanced_version_example ../../tests/test_toml_structure.toml
```

## Integration with Build System

These test files can be integrated into the CMake build system for automated testing:

```cmake
# Add to CMakeLists.txt
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
```

## Validation Guidelines

- **Version System:** Ensure version strings match CMake configuration
- **TOML Structure:** Validate all required sections are present
- **Compatibility:** Test with different HDMapping versions
- **Cross-Platform:** Verify tests work on Windows, Linux, macOS

---

*These tests help ensure HDMapping's configuration and versioning systems work correctly across different environments and use cases.*
