include(FetchContent)
FetchContent_Declare(mcap
        GIT_REPOSITORY https://github.com/foxglove/mcap.git
        GIT_TAG releases/cpp/v1.4.0   # pin a real tag
)
FetchContent_MakeAvailable(mcap)

add_library(mcap INTERFACE)
target_include_directories(mcap INTERFACE ${mcap_SOURCE_DIR}/cpp/mcap/include)

# mcap's reader/writer .inl always compile in LZ4/ZSTD (de)compression paths,
# even for callers that only ever write Compression::None -- so the symbols
# must be linked unconditionally, not just when compression is actually used.
#
# On Windows there's no system package manager providing liblz4/libzstd dev
# packages (unlike apt on Linux / Homebrew on macOS), and no vendored/prebuilt
# copy of them here yet -- see 3rdpartyBinary/Proj for that pattern if this
# ever needs revisiting. Until then, disable mcap's LZ4/ZSTD code paths on
# Windows via its own escape hatch: reading uncompressed/other-codec mcaps
# still works, only Compression::Lz4/Zstd at write time become unavailable --
# fine since this repo's writer (rosbags/McapWriter) only ever writes
# Compression::None.
if(WIN32)
    target_compile_definitions(mcap INTERFACE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
else()
    find_library(MCAP_LZ4_LIBRARY NAMES lz4)
    find_library(MCAP_ZSTD_LIBRARY NAMES zstd)
    find_path(MCAP_LZ4_INCLUDE_DIR NAMES lz4frame.h)
    find_path(MCAP_ZSTD_INCLUDE_DIR NAMES zstd.h)
    if(NOT MCAP_LZ4_LIBRARY OR NOT MCAP_ZSTD_LIBRARY OR NOT MCAP_LZ4_INCLUDE_DIR OR NOT MCAP_ZSTD_INCLUDE_DIR)
        message(FATAL_ERROR "mcap requires liblz4 and libzstd (dev packages) to build against")
    endif()
    target_link_libraries(mcap INTERFACE ${MCAP_LZ4_LIBRARY} ${MCAP_ZSTD_LIBRARY})
    target_include_directories(mcap INTERFACE ${MCAP_LZ4_INCLUDE_DIR} ${MCAP_ZSTD_INCLUDE_DIR})
endif()
