cmake_minimum_required(VERSION 3.28)

# ============================================================
# PROJ setup
# ============================================================

set(PROJ_VERSION v9.3.0)

# ----------------------------
# Linux / Unix
# ----------------------------
if(UNIX AND NOT WIN32)

    find_package(PROJ QUIET)

    if(NOT PROJ_FOUND)
        message(FATAL_ERROR
            "PROJ not found.\n"
            "Install it using:\n"
            "  sudo apt install libproj-dev\n"
            "Then reconfigure CMake."
        )
    endif()

endif()

# ----------------------------
# Windows – download prebuilt
# ----------------------------
if(WIN32)

    set(PROJ_FILENAME "proj-windows.zip")
    set(PROJ_URL
        "https://github.com/michalpelka/proj_win_deploy/releases/download/v9.3.0-2/proj-windows.zip"
    )

    # <- wklej SHA256 z release
    set(PROJ_SHA256
        "fc1fcbe1598f0c153febfbda6a5c068d4ddaef7d44d052dae55b23fbb4b66e28"
    )

    set(PROJ_DOWNLOAD_PATH "${CMAKE_BINARY_DIR}/${PROJ_FILENAME}")
    set(PROJ_EXTRACT_DIR   "${CMAKE_BINARY_DIR}/3rd_binary/PROJ")

    # ----------------------------
    # Download
    # ----------------------------
    if(NOT EXISTS "${PROJ_DOWNLOAD_PATH}")
        message(STATUS "Downloading PROJ ${PROJ_VERSION}...")
        file(DOWNLOAD
            "${PROJ_URL}"
            "${PROJ_DOWNLOAD_PATH}"
            EXPECTED_HASH SHA256=${PROJ_SHA256}
            SHOW_PROGRESS
            TLS_VERIFY ON
        )
    endif()

    # ----------------------------
    # Extract
    # ----------------------------
    if(NOT EXISTS "${PROJ_EXTRACT_DIR}/include")
        message(STATUS "Extracting PROJ...")
        file(ARCHIVE_EXTRACT
            INPUT "${PROJ_DOWNLOAD_PATH}"
            DESTINATION "${PROJ_EXTRACT_DIR}"
        )
    endif()
    set(PROJ_ROOT "${PROJ_EXTRACT_DIR}/install_proj")
    set (PROJ_INCLUDE_DIR "${PROJ_ROOT}/include")
    set (PROJ_LIBRARY "${PROJ_ROOT}/bin/proj_9_3.dll")
    set (PROJ_IMPLIB "${PROJ_ROOT}/lib/proj.lib")
    set (PROJ_DB "${PROJ_ROOT}/share/proj/proj.db")

    # ----------------------------
    # Create imported target
    # ----------------------------
    add_library(PROJ::proj SHARED IMPORTED)

    set_target_properties(PROJ::proj PROPERTIES
        IMPORTED_LOCATION "${PROJ_ROOT}/bin/proj_9_3.dll"
        IMPORTED_IMPLIB  "${PROJ_ROOT}/lib/proj_9_3.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${PROJ_ROOT}/include"
    )

endif()