#pragma once

#ifndef HDMAPPING_VERSION_MAJOR
#error "Major version not defined - define HDMAPPING_VERSION_MAJOR inside top-level CMakeLists.txt file."
#endif

#ifndef HDMAPPING_VERSION_MINOR
#error "Minor version not defined - define HDMAPPING_VERSION_MINOR inside top-level CMakeLists.txt file."
#endif

#ifndef HDMAPPING_VERSION_PATCH
#error "Minor version not defined - define HDMAPPING_VERSION_PATCH inside top-level CMakeLists.txt file."
#endif

#define HDMAPPING_DEFINITION_TO_STRING_X(definition) #definition
#define HDMAPPING_DEFINITION_TO_STRING(definition) HDMAPPING_DEFINITION_TO_STRING_X(definition)

#define HDMAPPING_VERSION_STRING                                                                                                           \
    "v" HDMAPPING_DEFINITION_TO_STRING(HDMAPPING_VERSION_MAJOR) "." HDMAPPING_DEFINITION_TO_STRING(                                        \
        HDMAPPING_VERSION_MINOR) "." HDMAPPING_DEFINITION_TO_STRING(HDMAPPING_VERSION_PATCH)
