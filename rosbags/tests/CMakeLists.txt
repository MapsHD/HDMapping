cmake_minimum_required(VERSION 4.0.0)

project(hdmapping_rosbags_tests)

# Unit tests for rosbags/ (LAZ/IMU <-> MCAP/ROS2-bag support code, both
# directions, the ROS1 bag reader/writer and the Ouster pcap reader). Uses doctest, same as shared/tests -- see that CMakeLists.txt for
# why. Only McapWriter.cpp defines MCAP_IMPLEMENTATION; McapReader.cpp relies on
# it being linked in alongside.
add_executable(hdmapping_rosbags_tests
    test_mcap_writer.cpp
    test_mcap_reader.cpp
    test_ouster_pcap_reader.cpp
    test_ros1_bag.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../McapWriter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../McapWriter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../McapReader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../McapReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../MandeyeSessionWriter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../MandeyeSessionWriter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../OusterPcapReader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../OusterPcapReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Ros1Bag.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../Ros1Bag.cpp
)

target_include_directories(hdmapping_rosbags_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${THIRDPARTY_DIRECTORY}/json/include
)

target_link_libraries(hdmapping_rosbags_tests PRIVATE mcap ros1bag_compression ${PLATFORM_LASZIP_LIB})

if (MSVC)
    target_compile_definitions(hdmapping_rosbags_tests PRIVATE _USE_MATH_DEFINES)
endif()

include(CTest)
add_test(NAME hdmapping_rosbags_tests COMMAND hdmapping_rosbags_tests)