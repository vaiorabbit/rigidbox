= RigidBox : A Small Library for 3D Rigid Body Physics Tutorial

                                             Last Update: Apr 26, 2020
                                                   Since: Feb 23, 2011
                                                   by vaiorabbit

* Introduction
* Features
* Confirmed environment
* How to build
* Samples
* Support for code reading
* Credit
* License


== Introduction

In recent we can easily get many helpful resources that tell us how to use rigid
body physics engines.  But when we want to know how to "implement" original one,
it's still rather hard to find.

Though there are some open-source physics engines available, they are too large
for newbies to understand the part and parcel of internal design - these
libraries have already evolved into large ones through many years of development.

The "part and paecel" of rigid body physics engine is divided into 3 parts:

1. Collision Detection : the process to check the intersection state between two rigid bodies.
2. Collision Response : the process to correct the motion of collided bodies.
3. Integration : the process to update the motion state (position, velocity) of rigid bodies.

The main work of rigid body physics engine is to execute the sequence of these
procedure (under the methods like Update(), StepSimulation(), etc.).

The RigidBox library is made to demonstrate the sequence (or the "part and
parcel") briefly, and designed as compact as possible for reader's convenience.



== Features

Simulates the rigid motion of boxes (cubes, cuboids, etc.).

To keep the source code simple, I adopted some implementation poicies listed below:

=== Box is the only shape available in this library

* It's simple : the code for collision detection only handles box-box case.
* You just want to know how to tilt your boxes, right?
* No spheres are available. It's too simple to implement, and you'll get no feeling of accomplishment.
* No planes are available. One big box can be used as a wall/floor, etc.

=== Quaternion-free

* To keep the required knowledge minimal, all rotational operations are described by 3x3 matrix.
  * Notice : In practice, quaternions are essential for production use (speeding up, reducing calculation error, etc.).

=== No advanced collision detection

* To keep the required knowledge minimal, collision detections are performed between each boxes (what we call 'brute-force').
  * Notice : In practice, one of broad-phase collision detection method should be implemented for speeding up.

=== No advanced solver for collision computation

* No iterative methods are used.
* Simplicity over stability.

=== Simplicity > Efficiency

* Most of mathematical functions are implemented with 'operator overloading' to make the appearance of the source code more 'formula-like'.
* STL containers are used to simplify the data management.


By these policies, 
  * RigidBox is not suitable for practical use.
  * But the whole source code of this library is fairly compact.

So RigidBox is suitable for programmers who wants to grasp the essencial parts of rigid body physics engine.



== Confirmed environment

* Mac OS X
  * 10.15, Xcode

* Windows
  * Windows 10 18363.778 Visual Studio 2019



== How to build

=== Prerequisites

==== CMake

* CMake can generate new project files / build scripts for your environment.
* http://www.cmake.org/

From the repository root, configure and build the library, tests, and demos:

* `cmake -S . -B build`
* `cmake --build build --config Release`
* `ctest --test-dir build -C Release --output-on-failure`

==== OpenGL

* Windows
  * gl.h : Install Windows SDK.
      
* Mac OS X
  * No setup required. The OS provides OpenGL-related headers/libraries
    as its standard feature.

* Linux (Ubuntu)
  * You can install OpenGL-related dev packages via package managers.

==== SDL3

When demos are enabled, CMake fetches and builds SDL 3.4.20 and Dear ImGui
1.91.9b. The SDL version is pinned in demo/CMakeLists.txt. OpenGL development
headers and libraries are required.


=== Tests and samples

* CTest runs CollisionTest, IntegrationTest, SolverTest, and EnvTest.
* CollisionDemo, DominoDemo, and StackDemo are included in the root build.
* To omit demos, pass `-DRIGIDBOX_BUILD_DEMOS=OFF` when configuring.



== Support for code reading

Readers are assumed to have basic knowledge/experience of these subjects:

* Mathematics
  * 3D vector and 3x3 matrix manipulation
  * The concept of world space coordinate / object space coordinate

* Physics
  * Motion of a point mass / Collision between two bodies

* C++
  * Specification (especially class and operator overloading)
  * Library (especially STL vector and iterator)



== Credit

* vaiorabbit (http://twitter.com/vaiorabbit)


 
== License

zlib/libpng license (see LICENSE.txt for details)
