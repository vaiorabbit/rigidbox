set( SolverTest_EXE_HDRS 
    ../common/TestFramework.h
    TCSolver.h
)

set( SolverTest_EXE_SRCS 
    SolverTest.cpp
)

include_directories( ../../include )
include_directories( ../common )

add_executable( SolverTest ${SolverTest_EXE_HDRS} ${SolverTest_EXE_SRCS} )
add_dependencies( SolverTest RigidBox )
target_link_libraries( SolverTest RigidBox_lib )
