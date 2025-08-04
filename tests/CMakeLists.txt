# HDMapping Tests CMakeLists.txt

cmake_minimum_required(VERSION 3.15.0)

# Only build tests if testing is enabled
if(BUILD_TESTING)
    enable_testing()
    
    # Version System Test
    add_executable(test_version_system 
        test_version_system.cpp
    )
    
    # Include paths for version system test
    target_include_directories(test_version_system PRIVATE
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1
        ${CMAKE_SOURCE_DIR}/shared/include
    )
    
    # Link version macros (already defined in main CMakeLists.txt)
    target_compile_definitions(test_version_system PRIVATE
        HDMAPPING_VERSION_MAJOR=${HDMAPPING_VERSION_MAJOR}
        HDMAPPING_VERSION_MINOR=${HDMAPPING_VERSION_MINOR}
        HDMAPPING_VERSION_PATCH=${HDMAPPING_VERSION_PATCH}
    )
    
    # Add test to CTest
    add_test(NAME version_system_test 
             COMMAND test_version_system
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
    
    # TOML Structure Validation Test (if enhanced_version_example exists)
    if(TARGET enhanced_version_example)
        add_test(NAME toml_structure_test
                 COMMAND enhanced_version_example ${CMAKE_CURRENT_SOURCE_DIR}/test_toml_structure.toml
                 WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )
        
        # Set test properties
        set_tests_properties(toml_structure_test PROPERTIES
            PASS_REGULAR_EXPRESSION "Config version: 0.85.0"
        )
    endif()
    
    # Set test properties
    set_tests_properties(version_system_test PROPERTIES
        PASS_REGULAR_EXPRESSION "Software version: [0-9]+\\.[0-9]+\\.[0-9]+"
    )
    
    # Copy TOML test file to build directory
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/test_toml_structure.toml
        ${CMAKE_CURRENT_BINARY_DIR}/test_toml_structure.toml
        COPYONLY
    )
    
    message(STATUS "HDMapping tests configured:")
    message(STATUS "  - Version system test: test_version_system")
    message(STATUS "  - TOML structure test: test_toml_structure.toml")
    
endif(BUILD_TESTING)
