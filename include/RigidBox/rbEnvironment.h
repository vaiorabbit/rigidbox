// -*- mode: C++; coding: utf-8; -*-
#ifndef RBENVIRONMENT_H_INCLUDED
#define RBENVIRONMENT_H_INCLUDED

#include <vector>
#include "rbSolver.h"
#include "rbTypes.h"

class rbEnvironment
{
public:

    typedef std::vector<rbRigidBody*> BodyPtrContainer;
    typedef std::vector<rbContact> ContactContainer;

    struct Config
    {
        rbs32 RigidBodyCapacity;
        rbs32 ContactCapacty;

        Config()
            : RigidBodyCapacity(10)
            , ContactCapacty(20)
            {}
    };

    rbEnvironment();
    rbEnvironment( const Config& config );
    ~rbEnvironment();


    rbRigidBody* RigidBody( rbu32 index )
        { return bodies.at( index ); }

    rbu32 RigidBodyCount()
        { return bodies.size(); }

    rbu32 RigidBodyCapacity()
        { return bodies.capacity(); }


    rbContact* Contact( rbu32 index )
        { return &contacts.at( index ); }

    rbu32 ContactCount()
        { return contacts.size(); }

    rbu32 ContactCapacity()
        { return contacts.capacity(); }


    bool Register( rbRigidBody* box );
    bool Unregister( rbRigidBody* box );

    void Update( rbReal dtime, int div );

private:

    BodyPtrContainer bodies;
    ContactContainer contacts;
    rbSolver solver;
    Config config;
};

// RigidBox : A Small Library for 3D Rigid Body Physics Tutorial
// Copyright (c) 2011- vaiorabbit <http://twitter.com/vaiorabbit>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
//     1. The origin of this software must not be misrepresented; you must not
//     claim that you wrote the original software. If you use this software
//     in a product, an acknowledgment in the product documentation would be
//     appreciated but is not required.
//
//     2. Altered source versions must be plainly marked as such, and must not be
//     misrepresented as being the original software.
//
//     3. This notice may not be removed or altered from any source
//     distribution.

#endif
