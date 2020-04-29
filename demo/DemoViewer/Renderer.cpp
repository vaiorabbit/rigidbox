// -*- mode: C++; coding: utf-8 -*-
#if defined(_MSC_VER)
# define WIN32_LEAN_AND_MEAN 
# include <Windows.h>
# include <GL/GL.h>
#elif defined(__APPLE__)
# include <OpenGL/gl.h>
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

static void RenderCube()
{
    // Cube Vertices
    static constexpr GLfloat cv[] =
        {
             0.5f, 0.5f, 0.5f,  // 0 :  0,  1,  2
            -0.5f, 0.5f, 0.5f,  // 1 :  3,  4,  5
            -0.5f,-0.5f, 0.5f,  // 2 :  6,  7,  8
             0.5f,-0.5f, 0.5f,  // 3 :  9, 10, 11
             0.5f,-0.5f,-0.5f,  // 4 : 12, 13, 14
             0.5f, 0.5f,-0.5f,  // 5 : 15, 16, 17
            -0.5f, 0.5f,-0.5f,  // 6 : 18, 19, 20
            -0.5f,-0.5f,-0.5f,  // 7 : 21, 22, 23
        };

    // Cube Normals
    static constexpr GLfloat cn[] =
        {
             0.0f, 0.0f, 1.0f,  // Face 0
             1.0f, 0.0f, 0.0f,  // Face 1
             0.0f, 1.0f, 0.0f,  // Face 2
            -1.0f, 0.0f, 0.0f,  // Face 3
             0.0f,-1.0f, 0.0f,  // Face 4
             0.0f, 0.0f,-1.0f,  // Face 5
        };

    // [MEMO]
    // Indices (Quad)
    //     0, 1, 2, 3,  // Face 0
    //     0, 3, 4, 5,  // Face 1
    //     0, 5, 6, 1,  // Face 2
    //     1, 6, 7, 2,  // Face 3
    //     7, 4, 3, 2,  // Face 4
    //     4, 7, 6, 5,  // Face 5
    // ↓Triangulate
    //     0,1,2, 0,2,3,  // Face 0
    //     0,3,4, 0,4,5,  // Face 1
    //     0,5,6, 0,6,1,  // Face 2
    //     1,6,7, 1,7,2,  // Face 3
    //     7,4,3, 7,3,2,  // Face 4
    //     4,7,6, 4,6,5,  // Face 5

    static constexpr GLfloat vertices[] = {
        // Face 0
        cv[ 0], cv[ 1], cv[ 2], // 0
        cv[ 3], cv[ 4], cv[ 5], // 1
        cv[ 6], cv[ 7], cv[ 8], // 2
        cv[ 9], cv[10], cv[11], // 3
        // Face 1
        cv[ 0], cv[ 1], cv[ 2], // 0
        cv[ 9], cv[10], cv[11], // 3
        cv[12], cv[13], cv[14], // 4
        cv[15], cv[16], cv[17], // 5
        // Face 2
        cv[ 0], cv[ 1], cv[ 2], // 0
        cv[15], cv[16], cv[17], // 5
        cv[18], cv[19], cv[20], // 6
        cv[ 3], cv[ 4], cv[ 5], // 1
        // Face 3
        cv[ 3], cv[ 4], cv[ 5], // 1
        cv[18], cv[19], cv[20], // 6
        cv[21], cv[22], cv[23], // 7
        cv[ 6], cv[ 7], cv[ 8], // 2
        // Face 4
        cv[21], cv[22], cv[23], // 7
        cv[12], cv[13], cv[14], // 4
        cv[ 9], cv[10], cv[11], // 3
        cv[ 6], cv[ 7], cv[ 8], // 2
        // Face 5
        cv[12], cv[13], cv[14], // 4
        cv[21], cv[22], cv[23], // 7
        cv[18], cv[19], cv[20], // 6
        cv[15], cv[16], cv[17], // 5
    };

    static constexpr GLfloat normals[] = {
        // Face 0
        cn[ 0], cn[ 1], cn[ 2],
        cn[ 0], cn[ 1], cn[ 2],
        cn[ 0], cn[ 1], cn[ 2],
        cn[ 0], cn[ 1], cn[ 2],
        // Face 1
        cn[ 3], cn[ 4], cn[ 5],
        cn[ 3], cn[ 4], cn[ 5],
        cn[ 3], cn[ 4], cn[ 5],
        cn[ 3], cn[ 4], cn[ 5],
        // Face 2
        cn[ 6], cn[ 7], cn[ 8],
        cn[ 6], cn[ 7], cn[ 8],
        cn[ 6], cn[ 7], cn[ 8],
        cn[ 6], cn[ 7], cn[ 8],
        // Face 3
        cn[ 9], cn[10], cn[11],
        cn[ 9], cn[10], cn[11],
        cn[ 9], cn[10], cn[11],
        cn[ 9], cn[10], cn[11],
        // Face 4
        cn[12], cn[13], cn[14],
        cn[12], cn[13], cn[14],
        cn[12], cn[13], cn[14],
        cn[12], cn[13], cn[14],
        // Face 5
        cn[15], cn[16], cn[17],
        cn[15], cn[16], cn[17],
        cn[15], cn[16], cn[17],
        cn[15], cn[16], cn[17],
    };

    static constexpr GLubyte indices[] = {
         0 + 0,  0 + 1,  0 + 2,   0 + 0,  0 + 2,  0 + 3,  // Face 0
         4 + 0,  4 + 1,  4 + 2,   4 + 0,  4 + 2,  4 + 3,  // Face 1
         8 + 0,  8 + 1,  8 + 2,   8 + 0,  8 + 2,  8 + 3,  // Face 2
        12 + 0, 12 + 1, 12 + 2,  12 + 0, 12 + 2, 12 + 3,  // Face 3
        16 + 0, 16 + 1, 16 + 2,  16 + 0, 16 + 2, 16 + 3,  // Face 4
        20 + 0, 20 + 1, 20 + 2,  20 + 0, 20 + 2, 20 + 3,  // Face 5
    };

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, vertices);
    glNormalPointer(GL_FLOAT, 0, normals);
    glDrawElements(GL_TRIANGLES, sizeof(indices)/sizeof(indices[0]), GL_UNSIGNED_BYTE, indices);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
};

void Renderer::RenderEnvironment( rbEnvironment* env )
{
    float mtxRT[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float mtxS[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

    for ( rbRigidBody* box : env->RigidBodies() )
    {
        if ( box->IsFixed() )
            continue;

        const rbVec3& pos = box->Position();
        const rbVec3& ext = box->HalfExtent();
        const rbMtx3& orn = box->Orientation();

        mtxS[0] = 2.0f*ext.e[0];  mtxS[5] = 2.0f*ext.e[1];  mtxS[10] = 2.0f*ext.e[2];

        mtxRT[0] = orn.Elem(0,0);  mtxRT[4] = orn.Elem(0,1);  mtxRT[ 8] = orn.Elem(0,2);  mtxRT[12] = pos.e[0];
        mtxRT[1] = orn.Elem(1,0);  mtxRT[5] = orn.Elem(1,1);  mtxRT[ 9] = orn.Elem(1,2);  mtxRT[13] = pos.e[1];
        mtxRT[2] = orn.Elem(2,0);  mtxRT[6] = orn.Elem(2,1);  mtxRT[10] = orn.Elem(2,2);  mtxRT[14] = pos.e[2];

        glPushMatrix();
        glMultMatrixf( mtxRT );
        glMultMatrixf( mtxS );
        RenderCube();
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
        for (rbContact& c : env->Contacts()) {
            glVertex3f( c.Position.e[0], c.Position.e[1], c.Position.e[2] );
        }
        glEnd();
        glPopAttrib();
    }
}
