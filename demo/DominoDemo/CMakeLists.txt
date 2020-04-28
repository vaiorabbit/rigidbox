set( DominoDemo_EXE_HDRS 
    ../DemoViewer/DemoViewer.h
)

set( DominoDemo_EXE_SRCS 
    DominoDemo.cpp
)

# include/link path
include_directories(
    ${OPENGL_INCLUDE_DIR}
	../../include
	../DemoViewer
)

if ( CMAKE_HOST_WIN32 )
    link_directories( ${CMAKE_CURRENT_LIST_DIR}/../../lib )
    link_directories( ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/lib )
elseif ( CMAKE_HOST_APPLE )
    link_directories( ${SDL2_LIBRARY} )
endif()

# define target
add_executable( DominoDemo ${DominoDemo_EXE_HDRS} ${DominoDemo_EXE_SRCS} )
add_dependencies( DominoDemo RigidBox_lib DemoViewer )

# libraries
if ( CMAKE_HOST_WIN32 )
    target_link_libraries( DominoDemo DemoViewer_lib RigidBox_lib ${OPENGL_LIBRARIES} SDL2 )
elseif ( CMAKE_HOST_APPLE )
    target_link_libraries( DominoDemo DemoViewer_lib RigidBox_lib ${OPENGL_LIBRARIES} )
    target_link_libraries( DominoDemo general ${SDL2_LIBRARY} )
endif()

set_target_properties(DominoDemo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if ( CMAKE_HOST_WIN32 )
    # "The file contains a character that cannot be represented in the current code page (...)"
    target_compile_options(DominoDemo PRIVATE "/wd4819")

    add_custom_command(TARGET DominoDemo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/SDL2/lib/SDL2.dll"
            $<TARGET_FILE_DIR:DominoDemo>)
endif()
