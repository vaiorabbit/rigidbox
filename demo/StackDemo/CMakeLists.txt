set( StackDemo_EXE_HDRS 
    ../DemoViewer/DemoViewer.h
)

set( StackDemo_EXE_SRCS 
    StackDemo.cpp
)

find_package(GLUT)
find_package(OpenGL)

include_directories( ${GLUT_INCLUDE_DIR} )
include_directories( ${OPENGL_INCLUDE_DIR} )
include_directories( ../../include )
include_directories( ../DemoViewer )

link_directories( ${CMAKE_CURRENT_LIST_DIR}/../../lib )

add_executable( StackDemo ${StackDemo_EXE_HDRS} ${StackDemo_EXE_SRCS} )
add_dependencies( StackDemo RigidBox DemoViewer )
target_link_libraries( StackDemo DemoViewer_lib RigidBox_lib ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )

set_target_properties(StackDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
