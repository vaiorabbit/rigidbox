add_executable(CollisionTest CollisionTest.cpp TCCollision.h ../common/TestFramework.h)
target_include_directories(CollisionTest PRIVATE ../common)
target_link_libraries(CollisionTest PRIVATE RigidBox)
add_test(NAME CollisionTest COMMAND CollisionTest)

if(MSVC)
    target_compile_options(CollisionTest PRIVATE "/wd4819")
endif()
