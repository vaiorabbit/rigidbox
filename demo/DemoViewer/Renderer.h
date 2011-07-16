// -*- mode: C++; coding: utf-8 -*-
#ifndef DEMOVIEWER_RENDERER_H_INCLUDED
#define DEMOVIEWER_RENDERER_H_INCLUDED

class rbEnvironment;

class Renderer
{
public:

    Renderer();
    ~Renderer();
    void Initialize();
    void Finalize();
    void RenderFloor();

    void RenderEnvironment( rbEnvironment* env );

private:

    void InitializeFloor();
    void FinalizeFloor();

    float* floor_verts;
    int floor_indices_count;
};

#endif
