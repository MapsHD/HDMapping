include_guard()

set(IMPLOT_LIBRARY_DIRECTORY ${THIRDPARTY_DIRECTORY}/implot)

set(IMPLOT_SOURCE_FILES
        ${IMPLOT_LIBRARY_DIRECTORY}/implot.cpp
        ${IMPLOT_LIBRARY_DIRECTORY}/implot_items.cpp
        ${IMPLOT_LIBRARY_DIRECTORY}/implot_demo.cpp
)

set(IMPLOT_HEADER_FILES
        ${IMPLOT_LIBRARY_DIRECTORY}/implot.h
        ${IMPLOT_LIBRARY_DIRECTORY}/implot_internal.h
)

set(IMPLOT_FILES ${IMPLOT_SOURCE_FILES} ${IMPLOT_HEADER_FILES})

add_library(implot STATIC)
target_sources(implot PRIVATE ${IMPLOT_FILES})
target_include_directories(
        implot PRIVATE ${IMPLOT_LIBRARY_DIRECTORY} ${IMPLOT_LIBRARY_BACKEND_DIRECTORY}
        ${THIRDPARTY_DIRECTORY}/imgui)
