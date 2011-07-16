// -*- mode: C++; coding: utf-8 -*-
#ifndef DEMOVIEWER_CAMERA_H_INCLUDED
#define DEMOVIEWER_CAMERA_H_INCLUDED

class Camera
{
public:

    static const unsigned int MouseButton_None   = 0x00000000U;
    static const unsigned int MouseButton_Left   = 0x00000001U;
    static const unsigned int MouseButton_Middle = 0x00000002U;
    static const unsigned int MouseButton_Right  = 0x00000004U;

    static const unsigned int MouseState_None = 0x00000000U;
    static const unsigned int MouseState_Up   = 0x00000001U;
    static const unsigned int MouseState_Down = 0x00000002U;

    Camera();
    ~Camera();

    int Width();
    void SetWidth( int width );

    int Height();
    void SetHeight( int height );

    void SetMouseState( unsigned int button, unsigned int state, int x, int y );
    void UpdateFromMouseMotion( int x, int y );

    void ViewMatrix( float mtxView[16] );
    void ProjectionMatrix( float mtxProj[16] );

private:

    void ClampParameters();
    void UpdateViewMatrix();
    void UpdateProjectionMatrix();

    float position[3], at[3], up[3];
    unsigned int mouse_button;
    int mouse_x_prev, mouse_y_prev;
    float phi, theta;
    float radius, radius_min, radius_max;
    int width, height;
    float fovy_deg, z_near, z_far;
    float mtxView[16], mtxProj[16];
    bool renew_view, renew_proj;
};

#endif
