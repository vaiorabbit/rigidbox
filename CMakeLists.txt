cmake_minimum_required( VERSION 3.17.1 )
project( RigidBox )

file( GLOB RIGIDBOX_LIB_HDRS "${CMAKE_CURRENT_LIST_DIR}/include/RigidBox/*.h" )
file( GLOB RIGIDBOX_LIB_SRCS "${CMAKE_CURRENT_LIST_DIR}/source/*.cpp" )
add_library( RigidBox ${RIGIDBOX_LIB_HDRS} ${RIGIDBOX_LIB_SRCS} )
include_directories( ${CMAKE_CURRENT_LIST_DIR}/include )

set( LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/lib )

set_target_properties(RigidBox PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if ( MSVC )
    # "The file contains a character that cannot be represented in the current code page (...)"
    target_compile_options(RigidBox PRIVATE "/wd4819")
endif()
