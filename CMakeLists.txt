cmake_minimum_required(VERSION 3.17)
project(RigidBox LANGUAGES CXX)

include(CTest)

file(GLOB RIGIDBOX_LIB_HDRS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/RigidBox/*.h")
file(GLOB RIGIDBOX_LIB_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")

add_library(RigidBox ${RIGIDBOX_LIB_HDRS} ${RIGIDBOX_LIB_SRCS})
target_include_directories(RigidBox PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_target_properties(RigidBox PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(RigidBox PRIVATE "/wd4819")
endif()

option(RIGIDBOX_BUILD_DEMOS "Build the SDL3 demos" ON)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

if(RIGIDBOX_BUILD_DEMOS)
    add_subdirectory(demo)
endif()
