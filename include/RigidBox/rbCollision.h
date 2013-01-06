// -*- mode: C++; coding: utf-8; -*-
#ifndef RBCOLLISION_H_INCLUDED
#define RBCOLLISION_H_INCLUDED

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

#endif
