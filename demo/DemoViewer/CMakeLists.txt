# Core sources
file( GLOB DEMOVIEWER_LIB_HDRS "${CMAKE_CURRENT_LIST_DIR}/*.h" )
file( GLOB DEMOVIEWER_LIB_SRCS "${CMAKE_CURRENT_LIST_DIR}/*.cpp" )

# ImGui
set(IMGUI_PATH "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/imgui/imgui")
file( GLOB IMGUI_SRCS
    "${IMGUI_PATH}/*.cpp"
    "${IMGUI_PATH}/*.h"
)
source_group("imgui" FILES ${IMGUI_SRCS})

# macOS
if (CMAKE_HOST_APPLE)
    list(APPEND CMAKE_FRAMEWORK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2")
    find_library(SDL2_LIBRARY NAMES SDL2)
    # find_package(SDL2 REQUIRED PATH ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2)
endif()

add_library( DemoViewer ${DEMOVIEWER_LIB_HDRS} ${DEMOVIEWER_LIB_SRCS} ${IMGUI_SRCS} )

# include/link path
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../../include
    ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/include
    ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/imgui/imgui
	${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/SDL2.framework/Headers
)
link_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../../lib
    ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/lib
)

add_dependencies( DemoViewer RigidBox )

set( LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/../lib )

set_target_properties(DemoViewer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if ( MSVC )
    # "The file contains a character that cannot be represented in the current code page (...)"
    target_compile_options(DemoViewer PRIVATE "/wd4819")
endif()
