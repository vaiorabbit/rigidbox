// -*- mode: C++; coding: utf-8; -*-
#include <algorithm>
#include <functional>

#include <RigidBox/rbCollision.h>
#include <RigidBox/rbEnvironment.h>
#include <RigidBox/rbMath.h>
#include <RigidBox/rbRigidBody.h>
#include <RigidBox/rbSolver.h>

rbEnvironment::rbEnvironment()
    : bodies()
    , contacts()
    , solver()
    , config()
{
    Config default_config;
    bodies.reserve( default_config.RigidBodyCapacity );
    contacts.reserve( default_config.ContactCapacty );
    this->config = default_config;
}

rbEnvironment::rbEnvironment( const Config& config )
    : bodies()
    , contacts()
    , solver()
    , config()
{
    bodies.reserve( config.RigidBodyCapacity );
    contacts.reserve( config.ContactCapacty );
    this->config = config;
}

rbEnvironment::~rbEnvironment()
{
    if ( !bodies.empty() )
    {
        BodyPtrContainer::iterator it_end = bodies.end();
        for ( BodyPtrContainer::iterator it_current = bodies.begin(); it_current != it_end; ++it_current )
        {
            delete *it_current;
        }
        bodies.clear();
    }
}


bool rbEnvironment::Register( rbRigidBody* box )
{
    BodyPtrContainer::iterator it = std::find( bodies.begin(), bodies.end(), box );

    if ( it == bodies.end() )
    {
        bodies.push_back( box );
        return true;
    }

    return false;
}

bool rbEnvironment::Unregister( rbRigidBody* box )
{
    BodyPtrContainer::iterator it = std::find( bodies.begin(), bodies.end(), box );

    if ( it != bodies.end() )
    {
        bodies.erase( it );
        return true;
    }

    return false;
}


struct SameContacts : public std::binary_function<const rbContact&, const rbContact&, bool>
{
    static constexpr rbReal NearThreshold = rbReal(0.02);

    bool operator()( const rbContact& c0, const rbContact& c1 ) const
    {
        return (c0.Position - c1.Position).LengthSq() <= NearThreshold ? true : false;
    }
};


void rbEnvironment::Update( rbReal dtime, int div )
{
    rbReal dt = dtime / div;

    BodyPtrContainer::iterator it_body, it_body0, it_body1, it_bodies_end = bodies.end();

    // [LANG en] Preprocess
    // [LANG ja] 前処理

    // [LANG en] Cleanup contact point array
    // [LANG ja] 衝突点配列をゼロクリア
    contacts.clear();

    for ( rbs32 i = 0; i < div; ++i )
    {
        for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
        {
            // [LANG en] Cleanup temporal space used by collision response routine
            // [LANG ja] 衝突応答で利用する一時領域をゼロクリア
            (*it_body)->ClearSolverWorkArea();

            // [LANG en] The orientation of +it_body+ might be modified in the previous loop. So inertia tensor must be updated here.
            // [LANG ja] 位置が更新されているため慣性テンソルも更新
            (*it_body)->UpdateInvInertiaWorld();
        }

        // [LANG en] Collision detection
        // [LANG ja] 衝突検出
        for ( it_body0 = bodies.begin(); it_body0 != it_bodies_end; ++it_body0 )
        {
            for ( it_body1 = it_body0 + 1; it_body1 != it_bodies_end; ++it_body1 )
            {
                // [LANG en] No need to check between wall/floor intersection
                // [LANG ja] 壁/床同士の衝突判定は不要
                if ( (*it_body0)->IsFixed() && (*it_body1)->IsFixed() )
                    continue;

                rbContact c;
                if ( rbCollision::Detect( *it_body0, *it_body1, &c ) > 0 )
                {
                    // [LANG en] No need to register if +contacts+ already have the same (or similar) contact point
                    // [LANG ja] すでに似た衝突点が検出済みである場合は登録しない
                    ContactContainer::iterator it = std::find_if( contacts.begin(), contacts.end(),
                        std::bind(SameContacts(), c, std::placeholders::_1) );
                    if ( it == contacts.end() )
                        contacts.push_back( c );
                }
            }
        }

        // [LANG en] Integration (Force -> Velocity)
        // [LANG ja] 積分 (力→速度)
        for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
            (*it_body)->UpdateVelocity( dt );

        // [LANG en] Collision response
        // [LANG ja] 衝突応答
        ContactContainer::iterator it_contacts_end = contacts.end();
        for ( ContactContainer::iterator it_contact = contacts.begin(); it_contact != it_contacts_end; ++it_contact )
            solver.ApplyImpulse( &(*it_contact), dt );

        for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
            (*it_body)->CorrectVelocity();

        // [LANG en] Update sleep status
        // [LANG ja] スリープ状態の更新
        for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
        {
            (*it_body)->UpdateSleepStatus( dt );
            if ( (*it_body)->Sleeping() )
            {
                (*it_body)->SetLinearVelocity( 0, 0, 0 );

                // [LANG en] Angular momentum is also cleared in this method
                // [LANG ja] Angular Momentum も内部でゼロクリアされる
                (*it_body)->SetAngularVelocity( 0, 0, 0 );
            }
        }

        // [LANG en] Integration (Velocity -> Position)
        // [LANG ja] 積分 (速度→位置)
        for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
            (*it_body)->UpdatePosition( dt );
    }

    // [LANG en] Postprocess
    // [LANG ja] 後処理
    for ( it_body = bodies.begin(); it_body != it_bodies_end; ++it_body )
    {
        (*it_body)->SetForce( 0, 0, 0 );
        (*it_body)->SetTorque( 0, 0, 0 );
    }
}

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
