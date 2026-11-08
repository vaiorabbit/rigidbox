add_executable(CollisionDemo CollisionDemo.cpp)
target_link_libraries(CollisionDemo PRIVATE DemoViewer)
set_target_properties(CollisionDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(CollisionDemo PRIVATE "/wd4819")
endif()

if(WIN32)
    add_custom_command(TARGET CollisionDemo POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:SDL3::SDL3>"
            "$<TARGET_FILE_DIR:CollisionDemo>"
    )
endif()
