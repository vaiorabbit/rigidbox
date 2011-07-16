cmake_minimum_required( VERSION 2.8.3 )
project( RigidBox )

file( GLOB RIGIDBOX_LIB_HDRS "${CMAKE_CURRENT_LIST_DIR}/include/RigidBox/*.h" )
file( GLOB RIGIDBOX_LIB_SRCS "${CMAKE_CURRENT_LIST_DIR}/source/*.cpp" )
add_library( RigidBox ${RIGIDBOX_LIB_HDRS} ${RIGIDBOX_LIB_SRCS} )
include_directories( ${CMAKE_CURRENT_LIST_DIR}/include )

set( LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/lib )
