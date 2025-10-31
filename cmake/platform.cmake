include_guard()

if(WIN32)
    set(PLATFORM_MISCELLANEOUS_LIBS TBB::tbb)
    set(PLATFORM_LASZIP_LIB laszip3)
else()
    set(PLATFORM_MISCELLANEOUS_LIBS TBB::tbb)
    set(PLATFORM_LASZIP_LIB laszip)
endif()