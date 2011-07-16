// -*- mode: C++; coding: utf-8 -*-
#ifndef DEMOVIEWER_SCREENSHOT_H_INCLUDED
#define DEMOVIEWER_SCREENSHOT_H_INCLUDED

#include <string>

class ScreenShotSession
{
    int width, height, depth;
    int count;
    std::string session_name;
    char* buffer;

    void UpdateSessionName();

public:

    ScreenShotSession( int width, int height, int depth );
    ~ScreenShotSession();
    void Save();
    void Reset( int width, int height, int depth );
};

#endif
