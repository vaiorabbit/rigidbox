file( GLOB DEMOVIEWER_LIB_HDRS "${CMAKE_CURRENT_LIST_DIR}/*.h" )
file( GLOB DEMOVIEWER_LIB_SRCS "${CMAKE_CURRENT_LIST_DIR}/*.cpp" )
add_library( DemoViewer ${DEMOVIEWER_LIB_HDRS} ${DEMOVIEWER_LIB_SRCS} )
include_directories( ${CMAKE_CURRENT_LIST_DIR}/../../include )

add_dependencies( DemoViewer RigidBox )

set( LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/../lib )
