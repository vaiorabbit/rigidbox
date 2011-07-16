// -*- mode: C++; coding: utf-8 -*-
#ifndef DEMOVIEWER_SCENE_H_INCLUDED
#define DEMOVIEWER_SCENE_H_INCLUDED

class Renderer;

class Scene
{
public:
    Scene() {}
    virtual ~Scene() {}

    virtual void Initialize() {}
    virtual void Update( float dt ) =0;
    virtual void Reset() =0;
    virtual void Render( Renderer* renderer ) =0;
    virtual void Finalize() {}
};

#endif
