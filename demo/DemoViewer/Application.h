// -*- mode: C++; coding: utf-8 -*-
#pragma once

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
    bool MainLoop();
    void Finalize();

private:

    struct WindowImpl;
    WindowImpl* window_impl;

    struct EventImpl;
    EventImpl* event_impl;

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

    void OnKeyboard();
    void OnMouse();
    void OnMotion();
    void OnReshape();
    void OnDisplay();
    void OnIdle();

    void Resize(int32_t width, int32_t height);
};
