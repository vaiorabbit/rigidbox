cmake_minimum_required( VERSION 2.8.3 )
project( RigidBoxTest )

if (CMAKE_HOST_WIN32)
    include_external_msproject( RigidBox ${CMAKE_CURRENT_LIST_DIR}/../build/RigidBox.vcxproj )
endif()

# IMPORTED variable :
# http://www.cmake.org/Wiki/CMake_2.6_Notes
add_library( RigidBox_lib STATIC IMPORTED )
if (CMAKE_HOST_WIN32)
   set_property( TARGET RigidBox_lib PROPERTY IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_LIST_DIR}/../lib/Debug/RigidBox.lib)
   set_property( TARGET RigidBox_lib PROPERTY IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_LIST_DIR}/../lib/Release/RigidBox.lib)
elseif (CMAKE_HOST_APPLE)
   set_property( TARGET RigidBox_lib PROPERTY IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_LIST_DIR}/../lib/Debug/libRigidBox.a)
   set_property( TARGET RigidBox_lib PROPERTY IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_LIST_DIR}/../lib/Release/libRigidBox.a)
elseif(CMAKE_HOST_UNIX)
   set_property( TARGET RigidBox_lib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../lib/libRigidBox.a)
endif()


add_subdirectory( CollisionTest )
add_subdirectory( IntegrationTest )
add_subdirectory( SolverTest )
add_subdirectory( EnvTest )
