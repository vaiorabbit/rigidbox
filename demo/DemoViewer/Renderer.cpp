// -*- mode: C++; coding: utf-8 -*-
#if defined(__APPLE__)
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif
#include <RigidBox/RigidBox.h>
#include "Renderer.h"

Renderer::Renderer()
    : floor_verts( NULL )
    , floor_indices_count( 0 )
{}

Renderer::~Renderer()
{}

void Renderer::Initialize()
{
    InitializeFloor();
}

void Renderer::Finalize()
{
    FinalizeFloor();
}

void Renderer::InitializeFloor()
{
    float pos_max = 10.0f;
    float pos_min = -pos_max;
    int nlines = 10;
    float stride = (pos_max - pos_min) / nlines;

    floor_verts = new float[ 2 * (3 * 2 * (nlines + 1)) ];
    floor_indices_count = 2 * 2 * (nlines + 1);

    for ( int i = 0; i <= nlines; ++i )
    {
        floor_verts[6*i+0] = pos_min;
        floor_verts[6*i+1] = 0.0;
        floor_verts[6*i+2] = pos_min+stride*i;

        floor_verts[6*i+3] = pos_max;
        floor_verts[6*i+4] = 0.0;
        floor_verts[6*i+5] = pos_min+stride*i;
    }

    for ( int i = 0; i <= nlines; ++i )
    {
        int j = i + nlines+1;

        floor_verts[6*j+0] = pos_min+stride*i;
        floor_verts[6*j+1] = 0.0;
        floor_verts[6*j+2] = pos_min;

        floor_verts[6*j+3] = pos_min+stride*i;
        floor_verts[6*j+4] = 0.0;
        floor_verts[6*j+5] = pos_max;
    }
}

void Renderer::FinalizeFloor()
{
    if ( floor_verts )
        delete floor_verts;
}

void Renderer::RenderFloor()
{
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    glLineWidth( 1.0f );
    glDisable( GL_LIGHTING );
    glShadeModel( GL_FLAT );
    glColor4f( 0.0f, 1.0f, 0.0f, 1.0f );

    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer( 3, GL_FLOAT, 0, floor_verts );
    glDrawArrays( GL_LINES, 0, floor_indices_count );
    glDisableClientState( GL_VERTEX_ARRAY );

    glPopAttrib();
}

void Renderer::RenderEnvironment( rbEnvironment* env )
{
    float mtxRT[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float mtxS[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

    rbu32 bodies_count = env->RigidBodyCount();
    for ( rbu32 i = 0; i < bodies_count; ++i )
    {
        rbRigidBody* box = env->RigidBody( i );
        if ( box->IsFixed() )
            continue;

        rbVec3 pos = box->Position();
        rbVec3 ext = box->HalfExtent();
        rbMtx3 orn = box->Orientation();

        mtxS[0] = 2.0f*ext[0];  mtxS[5] = 2.0f*ext[1];  mtxS[10] = 2.0f*ext[2];

        mtxRT[0] = orn.r[0].e[0];  mtxRT[4] = orn.r[0].e[1];  mtxRT[ 8] = orn.r[0].e[2];  mtxRT[12] = pos[0];
        mtxRT[1] = orn.r[1].e[0];  mtxRT[5] = orn.r[1].e[1];  mtxRT[ 9] = orn.r[1].e[2];  mtxRT[13] = pos[1];
        mtxRT[2] = orn.r[2].e[0];  mtxRT[6] = orn.r[2].e[1];  mtxRT[10] = orn.r[2].e[2];  mtxRT[14] = pos[2];

        glPushMatrix();
        glMultMatrixf( mtxRT );
        glMultMatrixf( mtxS );
        glutSolidCube( 1.0f );
        glPopMatrix();
    }

    if ( 1 ) // ( render_contact_point )
    {
        glPushAttrib( GL_ALL_ATTRIB_BITS );
        glDisable( GL_LIGHTING );
        glDisable( GL_DEPTH_TEST );
        glPointSize( 5.0f );
        glColor3f( 1,0,0 );
        glBegin( GL_POINTS );
        for ( rbu32 i = 0; i < env->ContactCount(); ++i )
        {
            rbContact* c = env->Contact( i );
            rbVec3 p = c->Position;
            glVertex3f( p.e[0], p.e[1], p.e[2] );
        }
        glEnd();
        glPopAttrib();
    }
}
