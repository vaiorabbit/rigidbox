add_executable(IntegrationTest IntegrationTest.cpp TCIntegration.h ../common/TestFramework.h)
target_include_directories(IntegrationTest PRIVATE ../common)
target_link_libraries(IntegrationTest PRIVATE RigidBox)
add_test(NAME IntegrationTest COMMAND IntegrationTest)

if(MSVC)
    target_compile_options(IntegrationTest PRIVATE "/wd4819")
endif()
