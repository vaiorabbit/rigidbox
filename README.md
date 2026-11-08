<!-- -*- mode:markdown; coding:utf-8; -*- -->

# RigidBox : A Small Library for 3D Rigid Body Physics Tutorial #

*   Created : 2011-02-23
*   Last modified : 2020-04-29


RigidBox is a small library for demonstrating how to implement 3D rigid body physics engine.

## Build

Configure from the repository root to generate the library, tests, and SDL3 demos
in one build:

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The demo build fetches SDL 3.4.20 and Dear ImGui 1.91.9b through CMake
FetchContent. Set `-DRIGIDBOX_BUILD_DEMOS=OFF` to build only the library and tests.

## Demo ##

A scene from StackDemo.cpp:

[![A scene from StackDemo.cpp](http://img.youtube.com/vi/_ur39JB87Yw/mqdefault.jpg)](https://www.youtube.com/watch?v=_ur39JB87Yw)
![](http://img.youtube.com/vi/_ur39JB87Yw/1.jpg)
![](http://img.youtube.com/vi/_ur39JB87Yw/2.jpg)
![](http://img.youtube.com/vi/_ur39JB87Yw/3.jpg)
