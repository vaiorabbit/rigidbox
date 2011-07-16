set( EnvTest_EXE_HDRS 
    ../common/TestFramework.h
    TCEnv.h
)

set( EnvTest_EXE_SRCS 
    EnvTest.cpp
)

include_directories( ../../include )
include_directories( ../common )

add_executable( EnvTest ${EnvTest_EXE_HDRS} ${EnvTest_EXE_SRCS} )
add_dependencies( EnvTest RigidBox )
target_link_libraries( EnvTest RigidBox_lib )
