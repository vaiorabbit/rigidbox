set( IntegrationTest_EXE_HDRS 
    ../common/TestFramework.h
    TCIntegration.h
)

set( IntegrationTest_EXE_SRCS 
    IntegrationTest.cpp
)

include_directories( ../../include )
include_directories( ../common )

add_executable( IntegrationTest ${IntegrationTest_EXE_HDRS} ${IntegrationTest_EXE_SRCS} )
add_dependencies( IntegrationTest RigidBox )
target_link_libraries( IntegrationTest RigidBox_lib )
