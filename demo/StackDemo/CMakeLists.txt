set( StackDemo_EXE_HDRS 
    ../DemoViewer/DemoViewer.h
)

set( StackDemo_EXE_SRCS 
    StackDemo.cpp
)

find_package(OpenGL)

include_directories( ${OPENGL_INCLUDE_DIR} )
include_directories( ../../include )
include_directories( ../DemoViewer )

link_directories( ${CMAKE_CURRENT_LIST_DIR}/../../lib )
link_directories( ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/lib )

add_executable( StackDemo ${StackDemo_EXE_HDRS} ${StackDemo_EXE_SRCS} )
add_dependencies( StackDemo RigidBox DemoViewer )
target_link_libraries( StackDemo DemoViewer_lib RigidBox_lib ${OPENGL_LIBRARIES} SDL2 )

set_target_properties(StackDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if ( MSVC )
    # "The file contains a character that cannot be represented in the current code page (...)"
    target_compile_options(StackDemo PRIVATE "/wd4819")

    add_custom_command(TARGET StackDemo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/lib/SDL2.dll"
            $<TARGET_FILE_DIR:CollisionDemo>)
endif()
