// -*- mode: C++; coding: utf-8 -*-
#ifndef DEMOVIEWER_APPLICATION_H_INCLUDED
#define DEMOVIEWER_APPLICATION_H_INCLUDED

class Camera;
class Renderer;
class Scene;
class ScreenShotSession;

class Application
{
public:

    Application();
    ~Application();

    void Initialize( int argc, char* argv[] );
    void RegisterScene( Scene* scene, bool cleanup_on_exit = false );
    void UnregisterScene();
    void Run();
    void Finalize();

private:

    Camera* camera;
    Renderer* renderer;
    Scene* scene;
    ScreenShotSession* screenshot_session;

    bool cleanup_scene_on_exit;

    bool pause;
    bool one_step;
    bool capture_screen;
    bool capture_movie;

    int window_width, window_height;
    char* window_title;
    static Application* self;
    static void OnKeyboard( unsigned char key, int x, int y );
    static void OnMouse( int button, int state, int x, int y );
    static void OnMotion( int x, int y );
    static void OnReshape( int width, int height );
    static void OnDisplay( void );
    static void OnIdle( void );
};

#endif
