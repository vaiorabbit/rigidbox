// -*- mode: C++; coding: utf-8 -*-
#if defined(_MSC_VER)
# define WIN32_LEAN_AND_MEAN 
# include <Windows.h>
# include <GL/GL.h>
#elif defined(__APPLE__)
# include <OpenGL/gl.h>
#endif
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include "Application.h"
#include "Camera.h"
#include "Renderer.h"
#include "Scene.h"
#include "ScreenShot.h"

#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_impl_opengl2.h>

struct Application::WindowImpl
{
    SDL_Window* window = nullptr;
    SDL_GLContext context = nullptr;
};

struct Application::EventImpl
{
    SDL_Event event;
    bool running;
    bool operating_hud;

    EventImpl()
        : event()
        , running(true)
        , operating_hud(false)
        {}
};

Application::Application()
    : window_impl(nullptr)
    , event_impl(nullptr)
    , camera(nullptr)
    , renderer(nullptr)
    , scene(nullptr)
    , screenshot_session(nullptr)
    , cleanup_scene_on_exit( false )
    , pause( true )
    , one_step( false )
    , capture_screen( false )
    , window_width( 1280 )
    , window_height( 720 )
    , window_title( "RigidBox Demo" )
{}

Application::~Application()
{
    if ( scene && cleanup_scene_on_exit )
    {
        scene->Finalize();
        delete scene;
    }

    delete screenshot_session;

    renderer->Finalize();
    delete renderer;

    delete camera;

    delete event_impl;
    delete window_impl;
}

void Application::Initialize( int argc, char* argv[] )
{
    window_impl = new WindowImpl;
    event_impl = new EventImpl;
    std::memset(&event_impl->event, 0, sizeof(SDL_Event));

    SDL_Init( SDL_INIT_VIDEO );
    SDL_Quit();

    SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
    SDL_GL_SetAttribute( SDL_GL_ACCELERATED_VISUAL, 1 );
    SDL_GL_SetAttribute( SDL_GL_RED_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_ALPHA_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 24 );
    SDL_GL_SetAttribute( SDL_GL_STENCIL_SIZE, 8 );

    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 4 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );

    SDL_Window* window = SDL_CreateWindow( "", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_width, window_height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE );
    SDL_GLContext context = SDL_GL_CreateContext( window );
    SDL_GL_MakeCurrent(window, context);
    SDL_GL_SetSwapInterval(1);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window, context);
    ImGui_ImplOpenGL2_Init();

    window_impl->window = window;
    window_impl->context = context;

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_POLYGON_SMOOTH);

    renderer = new Renderer();
    renderer->Initialize();

    camera = new Camera();

    screenshot_session = new ScreenShotSession(window_width, window_height, 4);

    Resize(window_width, window_height);
}

void Application::RegisterScene( Scene* scene, bool cleanup_on_exit )
{
    this->scene = scene;
    cleanup_scene_on_exit = cleanup_on_exit;

    scene->Reset();
}

void Application::UnregisterScene()
{
    scene = nullptr;
}

void Application::Finalize()
{
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext( window_impl->context );
    SDL_DestroyWindow( window_impl->window );
    SDL_Quit();
}

void Application::OnKeyboard()
{
    // ESCAPE or Q -> quit application
    switch (event_impl->event.type)
    {
    case SDL_KEYDOWN:
        switch (event_impl->event.key.keysym.sym) {
        case SDLK_ESCAPE:
        case SDLK_q: /* fallthrough */
            event_impl->running = false;
            break;
        }
        break;
    }

    // Other operations are permitted only when HUD is deactivated
    if (!event_impl->operating_hud) {
        switch (event_impl->event.type)
        {
        case SDL_KEYDOWN:
            switch (event_impl->event.key.keysym.sym) {
            case SDLK_SPACE:
                this->pause = !this->pause;
                break;
            case SDLK_r: // Reset
                this->scene->Reset();
                this->pause = true;
                this->one_step = false;
                break;
            case SDLK_s:
                this->one_step = true;
                this->pause = true;
                break;
            case SDLK_c:
                this->capture_screen = true;
                break;
            }
            break;
        }
    }
}

void Application::OnMouse()
{
    // Mouse operations are permitted only when HUD is deactivated
    if (event_impl->operating_hud) {
        return;
    }

    auto button_map = [&](int32_t sdl_mouse_button) -> uint32_t {
                           switch (sdl_mouse_button) {
                           case SDL_BUTTON_LEFT:   return Camera::MouseButton_Left;   break;
                           case SDL_BUTTON_MIDDLE: return Camera::MouseButton_Middle; break;
                           case SDL_BUTTON_RIGHT:  return Camera::MouseButton_Right;  break;
                           default: return 0; break;
                           }
                       };

    auto state_map = [&](int32_t sdl_mouse_event) -> uint32_t {
                          switch (sdl_mouse_event) {
                          case SDL_MOUSEBUTTONDOWN: return Camera::MouseState_Down; break;
                          case SDL_MOUSEBUTTONUP:   return Camera::MouseState_Up;   break;
                          default: return 0; break;
                          }
                      };

    switch (event_impl->event.type)
    {
    case SDL_MOUSEBUTTONDOWN: /* fallthrouth */
    case SDL_MOUSEBUTTONUP:
        // TODO handle event.state (SDL_PRESSED or SDL_RELEASED) and SDL_MouseWheelEvent
        if (event_impl->operating_hud) {
            this->camera->SetMouseState( 0, 0, event_impl->event.button.x, event_impl->event.button.y );
        } else {
            this->camera->SetMouseState( button_map(event_impl->event.button.button), state_map(event_impl->event.type), event_impl->event.button.x, event_impl->event.button.y );
        }
        break;
    default:
        break;
    }
}

void Application::OnMotion()
{
    // Mouse operations are permitted only when HUD is deactivated
    if (event_impl->operating_hud) {
        return;
    }

    switch (event_impl->event.type)
    {
    case SDL_MOUSEMOTION:
        this->camera->UpdateFromMouseMotion( event_impl->event.motion.x, event_impl->event.motion.y );
        break;
    default:
        break;
    }
}

void Application::Resize(int32_t width, int32_t height)
{
    this->window_width = width;
    this->window_height = height;

    this->camera->SetWidth( width );
    this->camera->SetHeight( height );
    glViewport( 0, 0, width, height );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    float mtxProj[16];
    this->camera->ProjectionMatrix( mtxProj );
    glMultMatrixf( mtxProj );
    glMatrixMode( GL_MODELVIEW );

    this->screenshot_session->Reset( width, height, 4 );
}


void Application::OnReshape()
{
    if (event_impl->event.type == SDL_WINDOWEVENT &&
        event_impl->event.window.event == SDL_WINDOWEVENT_RESIZED) {
        if (window_impl->window == SDL_GetWindowFromID(event_impl->event.window.windowID)) {
            Resize(event_impl->event.window.data1, event_impl->event.window.data2);
        }
    }
}

void Application::OnDisplay()
{
    glPushAttrib( GL_ALL_ATTRIB_BITS );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glPushMatrix();
    glLoadIdentity();

    {
        float pos[4] = { 2.5f, 0.0f, 5.0f, 1.0f };
        float dif[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        float spe[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        float amb[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        glLightfv( GL_LIGHT0, GL_POSITION, pos );
        glLightfv( GL_LIGHT0, GL_DIFFUSE,  dif );
        glLightfv( GL_LIGHT0, GL_SPECULAR, spe );
        glLightfv( GL_LIGHT0, GL_AMBIENT,  amb );
    }

    float mtxView[16];
    this->camera->ViewMatrix( mtxView );
    glMultMatrixf( mtxView );

    this->renderer->RenderFloor();

    this->scene->Render( this->renderer );

    glPopMatrix();
    glPopAttrib();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplSDL2_NewFrame(window_impl->window);
    ImGui::NewFrame();
    ImGui::Begin("RigidBox Console");
    ImGui::Text("Simulation Control");

    bool pause_state = this->pause;
    ImGui::Checkbox("Pause (Uncheck to run the sim)", &pause_state);
    bool pressed_onestep = ImGui::Button("One Step");
    bool pressed_reset = ImGui::Button("Reset Simulation");

    if ( this->pause != pause_state) {
        this->pause = !this->pause;
    }
    if (pressed_onestep) {
        this->one_step = true;
        this->pause = true;
    }
    if (pressed_reset) {
        this->scene->Reset();
        this->pause = true;
        this->one_step = false;
    }

    event_impl->operating_hud = ImGui::IsWindowFocused();
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    if ( this->capture_screen )
    {
        this->screenshot_session->Save();
        this->capture_screen = false;
    }
}

void Application::OnIdle()
{
    static const float dt = 1.0f / 60.0f;

    if ( this->one_step || !this->pause )
        this->scene->Update( dt );

    this->one_step = false;
}

bool Application::MainLoop()
{
    while( SDL_PollEvent( &event_impl->event ) )
    {
        OnKeyboard();
        OnMouse();
        OnMotion();
        OnReshape();
        if (!event_impl->running) {
            return false;
        }
    }

    OnIdle();
    OnDisplay();

    SDL_GL_SwapWindow( window_impl->window );
    SDL_Delay( 1 );

    return true;
}
