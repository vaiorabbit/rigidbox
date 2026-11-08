file(GLOB DEMOVIEWER_HDRS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB DEMOVIEWER_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(DemoViewer ${DEMOVIEWER_HDRS} ${DEMOVIEWER_SRCS})
target_include_directories(DemoViewer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(DemoViewer
    PUBLIC RigidBox
    PRIVATE RigidBoxImGui SDL3::SDL3 OpenGL::GL
)
set_target_properties(DemoViewer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(DemoViewer PRIVATE "/wd4819")
endif()
