include_guard()

set(IMGUI_LIBRARY_DIRECTORY ${THIRDPARTY_DIRECTORY}/imgui)
set(IMGUI_LIBRARY_BACKEND_DIRECTORY ${IMGUI_LIBRARY_DIRECTORY}/backends)

set(IMGUI_SOURCE_FILES
    ${IMGUI_LIBRARY_DIRECTORY}/imgui_demo.cpp
    ${IMGUI_LIBRARY_DIRECTORY}/imgui_draw.cpp
    ${IMGUI_LIBRARY_DIRECTORY}/imgui_tables.cpp
    ${IMGUI_LIBRARY_DIRECTORY}/imgui_widgets.cpp
    ${IMGUI_LIBRARY_DIRECTORY}/imgui.cpp
    ${IMGUI_LIBRARY_BACKEND_DIRECTORY}/imgui_impl_glut.cpp
    ${IMGUI_LIBRARY_BACKEND_DIRECTORY}/imgui_impl_opengl2.cpp)

set(IMGUI_HEADER_FILES
    ${IMGUI_LIBRARY_DIRECTORY}/imgui.h
    ${IMGUI_LIBRARY_BACKEND_DIRECTORY}/imgui_impl_glut.h
    ${IMGUI_LIBRARY_BACKEND_DIRECTORY}/imgui_impl_opengl2.h)

set(IMGUI_FILES ${IMGUI_SOURCE_FILES} ${IMGUI_HEADER_FILES})
add_definitions(-DImDrawIdx=unsigned\ int)
add_library(imgui STATIC ${IMGUI_FILES})
target_include_directories(
    imgui PRIVATE ${IMGUI_LIBRARY_DIRECTORY} ${IMGUI_LIBRARY_BACKEND_DIRECTORY}
    ${THIRDPARTY_DIRECTORY}/freeglut/include)
