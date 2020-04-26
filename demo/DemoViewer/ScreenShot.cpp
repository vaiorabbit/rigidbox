// -*- mode: C++; coding: utf-8 -*-
#if defined(_MSC_VER)
# define WIN32_LEAN_AND_MEAN 
# include <Windows.h>
# include <GL/GL.h>
# pragma warning( push )
# pragma warning( disable : 4996 )
#elif defined(__APPLE__)
# include <OpenGL/gl.h>
#endif

// https://github.com/KhronosGroup/OpenGL-Registry/blob/master/api/GL/glext.h
#ifndef GL_BGRA
# define GL_BGRA                           0x80E1
#endif
#ifndef GL_UNSIGNED_INT_8_8_8_8_REV
# define GL_UNSIGNED_INT_8_8_8_8_REV       0x8367
#endif

#include <cstdio>
#include <ctime>
#include <RigidBox/RigidBox.h>
#include "ScreenShot.h"

void ScreenShotSession::UpdateSessionName()
{
    time_t timer_session = std::time( NULL );
    tm* tm_session = std::gmtime( &timer_session );

    char buf[64];
    std::sprintf( buf, "ScreenShot_%4d%02d%02d%02d%02d%02d",
                  tm_session->tm_year + 1900,
                  tm_session->tm_mon + 1,
                  tm_session->tm_mday,
                  tm_session->tm_hour,
                  tm_session->tm_min,
                  tm_session->tm_sec
        );

    session_name = buf;
}

ScreenShotSession::ScreenShotSession( int width_, int height_, int depth_ )
    : width( width_ )
    , height( height_ )
    , depth( depth_ )
    , count( 0 )
    , session_name()
    , buffer( NULL )
{
    buffer = new char[ width * height * depth ];

    UpdateSessionName();
}

ScreenShotSession::~ScreenShotSession()
{
    delete[] buffer;
}

void ScreenShotSession::Save()
{
    glReadPixels( 0, 0, width, height,
                  GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, buffer );

    char buf[64];
    std::sprintf( buf, "_ID%05d.tga", count );
    std::string filename( session_name );
    filename += buf;

    FILE* fp = std::fopen( filename.c_str(), "wb" );

    // Ref.: http://www.organicbit.com/closecombat/formats/tga.html
    struct TGAHeader
    {
        rbs8  identsize;
        rbs8  colourmaptype;
        rbs8  imagetype;
        rbs16 colourmapstart;
        rbs16 colourmaplength;
        rbs8  colourmapbits;
        rbs16 xstart;
        rbs16 ystart;
        rbs16 image_width;
        rbs16 image_height;
        rbs8  image_bits_per_pixel;
        rbs8  descriptor;

        TGAHeader()
            : identsize(0), colourmaptype(0), imagetype(2)
            , colourmapstart(0), colourmaplength(0), colourmapbits(0)
            , xstart(0), ystart(0)
            , image_width(0), image_height(0), image_bits_per_pixel(0), descriptor(8)
            {}
    } tga_header;

    tga_header.image_width = width;
    tga_header.image_height = height;
    tga_header.image_bits_per_pixel = depth * 8;

    std::fwrite( &tga_header.identsize,            1, sizeof(rbs8),  fp );
    std::fwrite( &tga_header.colourmaptype,        1, sizeof(rbs8),  fp );
    std::fwrite( &tga_header.imagetype,            1, sizeof(rbs8),  fp );
    std::fwrite( &tga_header.colourmapstart,       1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.colourmaplength,      1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.colourmapbits,        1, sizeof(rbs8),  fp );
    std::fwrite( &tga_header.xstart,               1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.ystart,               1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.image_width,          1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.image_height,         1, sizeof(rbs16), fp );
    std::fwrite( &tga_header.image_bits_per_pixel, 1, sizeof(rbs8),  fp );
    std::fwrite( &tga_header.descriptor,           1, sizeof(rbs8),  fp );

    std::fwrite( buffer, depth, width*height, fp );

    std::fclose( fp );

    ++count;
}

void ScreenShotSession::Reset( int width_, int height_, int depth_ )
{
    width  = width_;
    height = height_;
    depth  = depth_;

    delete[] buffer;
    buffer = new char[ width * height * depth ];

    count = 0;

    UpdateSessionName();
}

#if defined(_MSC_VER)
# pragma warning(pop)
#endif