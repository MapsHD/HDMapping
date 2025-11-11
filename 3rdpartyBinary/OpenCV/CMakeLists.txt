cmake_minimum_required(VERSION 3.28)
# Script tries to find OpenCV in system for Linux, downloads OpenCV release for Windows
if(UNIX)
    find_package(OpenCV QUIET)
    if(NOT OpenCV_FOUND)
          message(FATAL_ERROR 
            "OpenCV not found. Please install it using your package manager:\n"
            "  sudo apt install libopencv-dev\n"
            "Then reconfigure CMake."
        )
    endif()
endif()

if (WIN32)
    # Download OpenCV precompiled self-extractig 
    set(OPENCV_DOWNLOAD_EXE_URL "https://github.com/opencv/opencv/releases/download/4.12.0/opencv-4.12.0-windows.exe")
    set(OPENCV_DOWNLOAD_EXE_SHA256 "b753b14d880b9bc8d89d6acd3b665c040baec0211078435432fcae117db707af")
    set(OPENCV_DOWNLOAD_EXE "${CMAKE_BINARY_DIR}/opencv-4.12.0-windows.exe")


    if(NOT EXISTS ${OPENCV_DOWNLOAD_EXE})
        message(STATUS "Downloading OpenCV...")
        file(DOWNLOAD ${OPENCV_DOWNLOAD_EXE_URL} ${OPENCV_DOWNLOAD_EXE} SHOW_PROGRESS)
        # Verify SHA256 checksum
        # file(SHA256 ${OPENCV_DOWNLOAD_EXE} OPENCV_COMPUTED_SHA256)
        # if(NOT OPENCV_COMPUTED_SHA256 STREQUAL OPENCV_DOWNLOAD_EXE_SHA256)
        #     message(FATAL_ERROR "SHA256 checksum mismatch for downloaded OpenCV executable.")
        # endif()
    endif()

    # Extract OpenCV exe
    set(OPENCV_EXTRACT_DIR "${CMAKE_BINARY_DIR}/3rd_binary/OpenCV")
    if(NOT EXISTS ${OPENCV_EXTRACT_DIR})
        message(STATUS "Extracting OpenCV...")
        file(ARCHIVE_EXTRACT
            INPUT ${OPENCV_DOWNLOAD_EXE}
            DESTINATION ${OPENCV_EXTRACT_DIR}
        )
    endif()
    if(EXISTS ${OPENCV_EXTRACT_DIR})
        set (OpenCV_DIR ${CMAKE_BINARY_DIR}/3rd_binary/OpenCV/opencv/build) 
    endif()
endif()
