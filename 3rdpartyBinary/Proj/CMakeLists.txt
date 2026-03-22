cmake_minimum_required(VERSION 3.28)

# ============================================================
# PROJ setup
# ============================================================



# ----------------------------
# Linux / Unix
# ----------------------------
if(UNIX AND NOT WIN32)

    find_package(PROJ CONFIG QUIET)

    # Some distros expose lowercase target
    if(TARGET PROJ::proj)
        set(PROJ_TARGET PROJ::proj)

    elseif(TARGET proj)
        set(PROJ_TARGET proj)

    else()
        message(FATAL_ERROR
            "PROJ not found.\n"
            "libproj-dev is installed but CMake could not locate the config package.\n"
            "Check CMAKE_PREFIX_PATH or PROJ_DIR."
        )
    endif()

endif()

# ----------------------------
# Windows – download prebuilt
# ----------------------------
if(WIN32)
    set(PROJ_FILENAME "proj-windows.zip")
    set(PROJ_URL
        "https://github.com/MapsHD/proj_win_deploy/releases/download/v9.3.0-5/proj-windows.zip"
    )

  
    set(PROJ_SHA256
        "2d0cef6a07ca4d3e83cb6ca89637fb62db1650006876305e8d6a2af21468ae48"
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
    set(PROJ_VERSION v9.3.0)
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
        IMPORTED_IMPLIB  "${PROJ_ROOT}/lib/proj.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${PROJ_ROOT}/include"
    )
    add_library(PROJ::ZLIB SHARED IMPORTED)
    set_target_properties(PROJ::ZLIB PROPERTIES
        IMPORTED_LOCATION "${PROJ_ROOT}/bin/z.dll"
        IMPORTED_IMPLIB  "${PROJ_ROOT}/lib/zs.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${PROJ_ROOT}/include"
    )

    set_property(TARGET PROJ::proj APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES PROJ::ZLIB
    )

endif()
