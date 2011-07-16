set( CollisionDemo_EXE_HDRS 
    ../DemoViewer/DemoViewer.h
)

set( CollisionDemo_EXE_SRCS 
    CollisionDemo.cpp
)

find_package(GLUT)
find_package(OpenGL)

include_directories( ${GLUT_INCLUDE_DIR} )
include_directories( ${OPENGL_INCLUDE_DIR} )
include_directories( ../../include )
include_directories( ../DemoViewer )

add_executable( CollisionDemo ${CollisionDemo_EXE_HDRS} ${CollisionDemo_EXE_SRCS} )
add_dependencies( CollisionDemo RigidBox DemoViewer )
target_link_libraries( CollisionDemo DemoViewer_lib RigidBox_lib ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
