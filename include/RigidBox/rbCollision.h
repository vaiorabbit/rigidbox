// -*- mode: C++; coding: utf-8; -*-
#pragma once

#include "rbTypes.h"
#include "rbMath.h"

struct rbContact
{
    rbVec3 Position;
    rbVec3 RelativeBodyPosition[2];
    rbRigidBody* Body[2];

    // [LANG en] We will assume the direction of Normal is always from Body[1] to Body[0]. rbCollision::Detect implements this convention.
    // [LANG ja] Normal の向きは常に Body[1] -> Body[0] であるものとします。このルールは rbCollision::Detect で実装されています。
    rbVec3 Normal;

    rbReal PenetrationDepth;

    rbContact()
        {}

    rbContact( const rbContact& other )
        {
            *this = other;
        }

    rbContact& operator =( const rbContact& other )
        {
            if ( this != &other )
            {
                Position = other.Position;
                RelativeBodyPosition[0] = other.RelativeBodyPosition[0];
                RelativeBodyPosition[1] = other.RelativeBodyPosition[1];
                Body[0] = other.Body[0];
                Body[1] = other.Body[1];
                Normal = other.Normal;
                PenetrationDepth = other.PenetrationDepth;
            }

            return *this;
        }
};

// Collision detection algorithm
class rbCollision
{
public:

    static rbs32 Detect( rbRigidBody* box0, rbRigidBody* box1, rbContact* contact_out );
};

// RigidBox : A Small Library for 3D Rigid Body Physics Tutorial
// Copyright (c) 2011-2020 vaiorabbit <http://twitter.com/vaiorabbit>
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
