add_executable(SolverTest SolverTest.cpp TCSolver.h ../common/TestFramework.h)
target_include_directories(SolverTest PRIVATE ../common)
target_link_libraries(SolverTest PRIVATE RigidBox)
add_test(NAME SolverTest COMMAND SolverTest)

if(MSVC)
    target_compile_options(SolverTest PRIVATE "/wd4819")
endif()
