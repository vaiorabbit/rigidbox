= RigidBox : A Small Library for 3D Rigid Body Physics Tutorial

                                             Last Update: Mar 24, 2012
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
  * Version  : Snow Leopard (10.6)
  * Compiler : Xcode 4

* Windows
  * Version  : Windows Vista
  * Compiler : Visual C++ 2010 + Windows SDK v7.0A

* Linux
  * Version  : Ubuntu 11.04
  * Compiler : GCC 4.5



== How to build

=== Prerequisites

==== CMake

[Notice] If you use Visual Studio 2010 or Xcode 4, you can skip this step.
         Just use the project files in the 'build' directory.

* CMake can generate new project files / build scripts for your environment.
* http://www.cmake.org/

Move to the 'build' directory. Then type:

* Windows
  * > cmake -G "Visual Sutdio 2010" ..

* Mac OS X
  * $ cmake -G "Xcode" ..

* Linux (Ubuntu)
  * $ cmake -G "Unix Makefiles" ..

==== OpenGL

* Windows
  * gl.h : Install Windows SDK.
    http://www.microsoft.com/download/en/details.aspx?displaylang=en&id=8279
  * glext.h : Get the newest version from OpenGL.org.
    http://www.opengl.org/registry/api/glext.h
      
* Mac OS X
  * No setup required. The OS provides OpenGL-related headers/libraries
    as its standard feature.

* Linux (Ubuntu)
  * You can install OpenGL-related dev packages via package managers like
    Synaptic, etc.

==== GLUT

* Windows
  * I used FreeGLUT.
    http://freeglut.sourceforge.net/
    
* Mac OS X
  * No setup required. The OS provides OpenGL-related headers/libraries
    as its standard feature.

* Linux (Ubuntu)
  * You can install OpenGL-related dev packages via package managers like
    Synaptic, etc.


=== Building the library

* Windows
  * Use RigidBox.sln in the 'build' directory.
  * You'll get 'RigidBox.lib' at lib/Debug and lib/Release.

* Mac OS X
  * Use RigidBox.xcodeproj in the 'build' directory.
  * You'll get RigidBox.dylib at:
    ex.) /Users/[your account name]/Library/Developer/Xcode/DerivedData/ [new line] 
            RigidBoxDemo-egcoaigiqbgfwabxxwpufqtasprk/Build/Products/Debug/RigidBox.dylib

* Linux (Ubuntu)
  * Just 'make' at the 'build' directory.
  * You'll get libRigidBox.a at lib/ .



== Samples

* See CollisionDemo and StackDemo in the 'demo' directory.

* DemoViewer is the framework for sample programs (based on OpenGL/GLUT).

* Building the samples

  * If you need to create new build script, use CMake at the 'build' directory.

  * Windows
    * Use RigidBoxDemo.sln in the 'build' directory.
      Check CollisionDemo (or StackDemo) is specified as 'Startup Project'.

  * Mac OS X
    * Use RigidBoxDemo.xcworkspace in the 'build' directory.
      Check CollisionDemo (or StackDemo) is selected as the current Scheme.

  * Linux (Ubuntu)
    * Just 'make' at the 'build' directory.
      Executables will be made as build/CollisionDemo/CollisionDemo, etc.



== Support for code reading

[2012-03-24] (work-in-progress) implementation notes are available at:

  http://code.google.com/p/rigidbox/wiki/ImplementationNotes
  http://code.google.com/p/rigidbox/w/list

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
