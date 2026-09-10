include_guard()

# Prefix applied to installed executable names, so that generic tool names
# (laz_to_ply, matrix_mul, pcd_to_laz, ...) do not collide with unrelated
# binaries once they land in a shared directory like /usr/local/bin.
set(HDMAPPING_BINARY_PREFIX "hdmapping-" CACHE STRING
    "Prefix applied to installed executable names")

# hdmapping_install_app(<target>...)
#
# Installs each target into <prefix>/bin as "${HDMAPPING_BINARY_PREFIX}<name>",
# leaving the build-tree name alone. Keeping build/bin/ unprefixed is
# deliberate: deploy_mandeye.bat validates the build output against a hardcoded
# list of plain .exe names, and IDE run configurations reference them too.
#
# install(TARGETS) has no RENAME option, and the obvious workaround --
# install(PROGRAMS $<TARGET_FILE:t> RENAME ...) -- is wrong here: it copies the
# binary verbatim, so the installed executable keeps the build-tree RUNPATH
# (build/3rdparty/LASzip/lib) instead of having it stripped, and would resolve
# the bundled liblaszip.so from the build directory rather than the system
# liblaszip that CPACK_DEBIAN_PACKAGE_DEPENDS declares. So install(TARGETS)
# stays responsible for the copy (and its RPATH rewriting), and the file is
# renamed immediately afterwards by an install-time script.
#
# $<TARGET_FILE_NAME:> (not the target name) carries the platform executable
# suffix, so the installed Windows name keeps its .exe. DESTDIR has to be
# honoured explicitly, the way CMake's own generated cmake_install.cmake does,
# because CPack stages the DEB through it.
function(hdmapping_install_app)
    foreach(target_name IN LISTS ARGN)
        if(NOT TARGET ${target_name})
            message(FATAL_ERROR "hdmapping_install_app: no such target '${target_name}'")
        endif()

        install(TARGETS ${target_name} RUNTIME DESTINATION bin)

        # Declared after the install(TARGETS) above and in the same directory,
        # so it runs after it.
        install(CODE "
            set(_hdmapping_bindir \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin\")
            set(_hdmapping_plain \"\${_hdmapping_bindir}/$<TARGET_FILE_NAME:${target_name}>\")
            set(_hdmapping_prefixed \"\${_hdmapping_bindir}/${HDMAPPING_BINARY_PREFIX}$<TARGET_FILE_NAME:${target_name}>\")
            if(EXISTS \"\${_hdmapping_plain}\")
                file(REMOVE \"\${_hdmapping_prefixed}\")
                file(RENAME \"\${_hdmapping_plain}\" \"\${_hdmapping_prefixed}\")
                message(STATUS \"Installing: \${_hdmapping_prefixed}\")
            endif()
        ")
    endforeach()
endfunction()
