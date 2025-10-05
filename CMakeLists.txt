cmake_minimum_required(VERSION 3.10)
project(WheelLoadDistributor)

set(CMAKE_CXX_STANDARD 17)

# Paths
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/devTools/imgui)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/devTools/implot)

# GLFW and OpenGL
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
)

# ImGui sources
file(GLOB IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# ImPlot sources
file(GLOB IMPLOT_SOURCES
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

# Executable
add_executable(WheelLoadDistributor
    main.cpp
    axleLoads.cpp
    plots.cpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

# Link libraries
# Replace Windows libraries with Linux equivalents (if needed)
if (UNIX AND NOT APPLE)
    target_link_libraries(WheelLoadDistributor
        OpenGL::GL
        glfw
        X11
        pthread
        dl
        m
    )
endif()

# On macOS, explicitly link OpenGL as a framework
if(APPLE)
    target_link_libraries(${PROJECT_NAME}
        "-framework OpenGL"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
        glfw
    )
else()
    target_link_libraries(${PROJECT_NAME}
        OpenGL::GL
        glfw
    )
endif()

target_include_directories(WheelLoadDistributor
    PRIVATE
    devTools/imgui
    devTools/implot
)
