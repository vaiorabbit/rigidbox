add_executable(DominoDemo DominoDemo.cpp)
target_link_libraries(DominoDemo PRIVATE DemoViewer)
set_target_properties(DominoDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(DominoDemo PRIVATE "/wd4819")
endif()

if(WIN32)
    add_custom_command(TARGET DominoDemo POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:SDL3::SDL3>"
            "$<TARGET_FILE_DIR:DominoDemo>"
    )
endif()
