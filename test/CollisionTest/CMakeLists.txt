set( CollisionTest_EXE_HDRS 
    ../common/TestFramework.h
    TCCollision.h
)

set( CollisionTest_EXE_SRCS 
    CollisionTest.cpp
)

include_directories( ../../include )
include_directories( ../common )

add_executable( CollisionTest ${CollisionTest_EXE_HDRS} ${CollisionTest_EXE_SRCS} )
add_dependencies( CollisionTest RigidBox )
target_link_libraries( CollisionTest RigidBox_lib )
