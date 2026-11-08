add_executable(EnvTest EnvTest.cpp TCEnv.h ../common/TestFramework.h)
target_include_directories(EnvTest PRIVATE ../common)
target_link_libraries(EnvTest PRIVATE RigidBox)
add_test(NAME EnvTest COMMAND EnvTest)

if(MSVC)
    target_compile_options(EnvTest PRIVATE "/wd4819")
endif()
