include(FetchContent)
find_package(OpenGL REQUIRED)

set(RIGIDBOX_SDL_VERSION "3.4.20")
set(SDL_SHARED ON)
set(SDL_STATIC OFF)
set(SDL_INSTALL OFF)
set(SDL_TESTS OFF)
set(SDL_EXAMPLES OFF)

FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG "release-${RIGIDBOX_SDL_VERSION}"
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(SDL3)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9b
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(RigidBoxImGui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp"
)
target_include_directories(RigidBoxImGui PUBLIC
    "${imgui_SOURCE_DIR}"
    "${imgui_SOURCE_DIR}/backends"
)
target_link_libraries(RigidBoxImGui PRIVATE SDL3::SDL3)
set_target_properties(RigidBoxImGui PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

add_subdirectory(DemoViewer)
add_subdirectory(CollisionDemo)
add_subdirectory(DominoDemo)
add_subdirectory(StackDemo)
