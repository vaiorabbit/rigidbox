add_executable(StackDemo StackDemo.cpp)
target_link_libraries(StackDemo PRIVATE DemoViewer)
set_target_properties(StackDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(StackDemo PRIVATE "/wd4819")
endif()

if(WIN32)
    add_custom_command(TARGET StackDemo POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:SDL3::SDL3>"
            "$<TARGET_FILE_DIR:StackDemo>"
    )
endif()
