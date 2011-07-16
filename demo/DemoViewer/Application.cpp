// -*- mode: C++; coding: utf-8 -*-
#if defined(__APPLE__)
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "Application.h"
#include "Camera.h"
#include "Renderer.h"
#include "Scene.h"
#include "ScreenShot.h"

// #define ENABLE_CAPTURE_MOVIE

// static
Application* Application::self = NULL;

Application::Application()
    : camera( NULL )
    , renderer( NULL )
    , scene( NULL )
    , screenshot_session( NULL )
    , cleanup_scene_on_exit( false )
    , pause( true )
    , one_step( false )
    , capture_screen( false )
    , capture_movie( false )
    , window_width( 640 )
    , window_height( 340 )
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
}

void Application::Initialize( int argc, char* argv[] )
{
    self = this;

    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE );
    glutInitWindowPosition( 0, 0 );
    glutInitWindowSize( window_width, window_height );
    glutCreateWindow( window_title );

    glutKeyboardFunc( Application::OnKeyboard );
    glutMouseFunc( Application::OnMouse );
    glutMotionFunc( Application::OnMotion );
    glutReshapeFunc( Application::OnReshape );
    glutDisplayFunc( Application::OnDisplay );
    glutIdleFunc( Application::OnIdle );

    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );

    glEnable( GL_DEPTH_TEST );
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );

    renderer = new Renderer();
    renderer->Initialize();

    camera = new Camera();

    screenshot_session = new ScreenShotSession( window_width, window_height, 4 );
}

void Application::RegisterScene( Scene* scene, bool cleanup_on_exit )
{
    this->scene = scene;
    cleanup_scene_on_exit = cleanup_on_exit;

    scene->Reset();
}

void Application::UnregisterScene()
{
    scene = NULL;
}

void Application::Run()
{
    glutMainLoop();
}

void Application::Finalize()
{}

// static
void Application::OnKeyboard( unsigned char key, int x, int y )
{
    switch( key )
    {
    case 'r': // Reset
        self->scene->Reset();
        self->pause = true;
        self->one_step = false;
        break;

    case ' ':
        self->pause = !self->pause;
        break;

    case 's':
        self->one_step = true;
        self->pause = true;
        break;

    case 'c':
        self->capture_screen = true;
        break;

    case 'm':
        self->capture_movie = !self->capture_movie;
        break;

    case 'q': // Quit
    case 27:  // Esc
        exit( EXIT_SUCCESS );
        break;
    }
}

// static
void Application::OnMouse( int button, int state, int x, int y )
{
    static const unsigned int button_map[] = {
        Camera::MouseButton_Left,   // GLUT_LEFT_BUTTON
        Camera::MouseButton_Middle, // GLUT_MIDDLE_BUTTON
        Camera::MouseButton_Right   // GLUT_RIGHT_BUTTON
    };

    static const unsigned int state_map[] = {
        Camera::MouseState_Down, // GLUT_DOWN
        Camera::MouseState_Up    // GLUT_UP
    };

    self->camera->SetMouseState( button_map[button], state_map[state], x, y );
}

// static
void Application::OnMotion( int x, int y )
{
    self->camera->UpdateFromMouseMotion( x, y );
}

// static
void Application::OnReshape( int width, int height )
{
    self->window_width = width;
    self->window_height = height;

    self->camera->SetWidth( width );
    self->camera->SetHeight( height );
    glViewport( 0, 0, width, height );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    float mtxProj[16];
    self->camera->ProjectionMatrix( mtxProj );
    glMultMatrixf( mtxProj );
    glMatrixMode( GL_MODELVIEW );

    glutPostRedisplay();

    self->screenshot_session->Reset( width, height, 4 );
}

// static
void Application::OnDisplay( void )
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
    self->camera->ViewMatrix( mtxView );
    glMultMatrixf( mtxView );

    self->renderer->RenderFloor();

    self->scene->Render( self->renderer );

    glPopMatrix();
    glPopAttrib();

#ifdef ENABLE_CAPTURE_MOVIE
    if ( self->capture_movie && !self->capture_screen )
        self->capture_screen = true;
#endif // ENABLE_CAPTURE_MOVIE

    if ( self->capture_screen )
    {
        self->screenshot_session->Save();
        self->capture_screen = false;
    }

    glutSwapBuffers();
}

// static
void Application::OnIdle( void )
{
    static const float dt = 1.0f / 60.0f;

    if ( self->one_step || !self->pause )
        self->scene->Update( dt );

    self->one_step = false;

    glutPostRedisplay();
}
