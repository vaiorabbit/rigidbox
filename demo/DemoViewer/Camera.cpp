// -*- mode: C++; coding: utf-8 -*-
#include <cmath>
#include <cstring>
#include "Camera.h"

static const float Cam_Pi = float(3.141592653589793);

static inline void Vec3Add( float out[3], float a[3], float b[3] )
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

static inline void Vec3Sub( float out[3], float a[3], float b[3] )
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static inline float Vec3Dot( float a[3], float b[3] )
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline void Vec3Cross( float out[3], float a[3], float b[3] )
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static inline void Vec3Normalize( float a[3] )
{
    float length = sqrtf( Vec3Dot(a, a) );

    a[0] /= length;
    a[1] /= length;
    a[2] /= length;
}

Camera::Camera()
    : mouse_button( MouseState_None )
    , mouse_x_prev( 0 )
    , mouse_y_prev( 0 )
    , phi( Cam_Pi / 2.0f )
    , theta( Cam_Pi / 3.0f )
    , radius( 20.0f )
    , radius_min( 5.0f )
    , radius_max( 50.0f )
    , width( 640 )
    , height( 340 )
    , fovy_deg( 30.0f )
    , z_near( 0.1f )
    , z_far( 200.0f )
    , renew_view( true )
    , renew_proj( true )
{
    ClampParameters();

    position[0] = radius * std::sin(theta) * std::cos(phi);
    position[1] = radius * std::cos(theta);
    position[2] = radius * std::sin(theta) * std::sin(phi);

    at[0] = 0;  at[1] = 2;  at[2] = 0;
    up[0] = 0;  up[1] = 1;  up[2] = 0;

    UpdateViewMatrix();
    UpdateProjectionMatrix();
}

Camera::~Camera()
{}

int Camera::Width()
{
    return width;
}

void Camera::SetWidth( int width )
{
    this->width = width;
    renew_proj = true;
}

int Camera::Height()
{
    return height;
}

void Camera::SetHeight( int height )
{
    this->height = height;
    renew_proj = true;
}

void Camera::SetMouseState( unsigned int button, unsigned int state, int x, int y )
{
    switch ( state )
    {
    case MouseState_Down:
    {
        if ( (button & MouseButton_Left) != 0 )
            mouse_button |= MouseButton_Left;
        if ( (button & MouseButton_Middle) != 0 )
            mouse_button |= MouseButton_Middle;
        if ( (button & MouseButton_Right) != 0 )
            mouse_button |= MouseButton_Right;
    }
    break;

    case MouseState_Up:
    {
        if ( (mouse_button & MouseButton_Left) != 0 && (button & MouseButton_Left) != 0 )
            mouse_button &= ~MouseButton_Left;
        if ( (mouse_button & MouseButton_Middle) != 0 && (button & MouseButton_Middle) != 0 )
            mouse_button &= ~MouseButton_Middle;
        if ( (mouse_button & MouseButton_Right) != 0 && (button & MouseButton_Right) != 0 )
            mouse_button &= ~MouseButton_Right;
    }
    break;
    }

    mouse_x_prev = x;
    mouse_y_prev = y;
}

void Camera::UpdateFromMouseMotion( int x, int y )
{
    if ( mouse_button != 0 )
    {
        float dx = float( x - mouse_x_prev );
        float dy = float( y - mouse_y_prev );

        if ( (mouse_button & MouseButton_Left) != 0 )
        {
            float scale = 0.5f;
            phi += scale * dx * Cam_Pi / 180.0f;
            theta -= scale * dy * Cam_Pi / 180.0f;
        }
        else if ( (mouse_button & MouseButton_Right) != 0 )
        {
            float scale = 0.05f;
            radius -= scale * dy;
        }

        ClampParameters();
        position[0] = radius * std::sin(theta) * std::cos(phi);
        position[1] = radius * std::cos(theta);
        position[2] = radius * std::sin(theta) * std::sin(phi);

        Vec3Add( position, position, at );

        renew_view = true;
        renew_proj = true;
    }

    mouse_x_prev = x;
    mouse_y_prev = y;
}

void Camera::ViewMatrix( float mtxView[16] )
{
    if ( renew_view )
    {
        UpdateViewMatrix();
        renew_view = false;
    }
    std::memcpy( mtxView, this->mtxView, 16*sizeof(float) );
}

void Camera::ProjectionMatrix( float mtxProj[16] )
{
    if ( renew_proj )
    {
        UpdateProjectionMatrix();
        renew_proj = false;
    }
    std::memcpy( mtxProj, this->mtxProj, 16*sizeof(float) );
}

void Camera::ClampParameters()
{
    if ( radius > radius_max )
        radius = radius_max;
    if ( radius < radius_min )
        radius = radius_min;

    if ( phi > 2 * Cam_Pi )
        phi -= 2 * Cam_Pi;
    if ( phi < -2 * Cam_Pi )
        phi += 2 * Cam_Pi;

    static const float theta_lower_limit = 1.0e-6f;
    static const float theta_upper_limit = 2.0f * Cam_Pi / 3.0f;
    if ( theta > theta_upper_limit )
        theta = theta_upper_limit;
    if ( theta < theta_lower_limit )
        theta = theta_lower_limit;
}

void Camera::UpdateViewMatrix()
{
    float a_x[3], a_y[3], a_z[3];

    // Axis z
    Vec3Sub( a_z, position, at );
    Vec3Normalize( a_z );

    // Axis x
    Vec3Cross( a_x, up, a_z );
    Vec3Normalize( a_x );

    // Axis y
    Vec3Cross( a_y, a_z, a_x );

    mtxView[ 0] = a_x[0];  mtxView[ 4] = a_x[1];  mtxView[ 8] = a_x[2];  mtxView[12] = -Vec3Dot( a_x, position );
    mtxView[ 1] = a_y[0];  mtxView[ 5] = a_y[1];  mtxView[ 9] = a_y[2];  mtxView[13] = -Vec3Dot( a_y, position );
    mtxView[ 2] = a_z[0];  mtxView[ 6] = a_z[1];  mtxView[10] = a_z[2];  mtxView[14] = -Vec3Dot( a_z, position );
    mtxView[ 3] = 0;       mtxView[ 7] = 0;       mtxView[11] = 0;       mtxView[15] = 1;
}

void Camera::UpdateProjectionMatrix()
{
    float f = std::tan( 0.5f * (Cam_Pi * fovy_deg) / 180.0f );
    f = 1.0f / f;
    float aspect = float(width) / float(height);

    mtxProj[ 0] = f / aspect;  mtxProj[ 4] = 0;  mtxProj[ 8] = 0;                              mtxProj[12] = 0;
    mtxProj[ 1] = 0;           mtxProj[ 5] = f;  mtxProj[ 9] = 0;                              mtxProj[13] = 0;
    mtxProj[ 2] = 0;           mtxProj[ 6] = 0;  mtxProj[10] = (z_far+z_near)/(z_near-z_far);  mtxProj[14] = 2*z_far*z_near / (z_near-z_far);
    mtxProj[ 3] = 0;           mtxProj[ 7] = 0;  mtxProj[11] = -1.0f;                          mtxProj[15] = 0;
}
