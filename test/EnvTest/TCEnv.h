// -*- mode: C++; coding: utf-8; -*-
#ifndef TCENV_H_INCLUDED
#define TCENV_H_INCLUDED

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <RigidBox/RigidBox.h>
#include <TestFramework.h>

class TCEnv : public Test::Case
{
public:
    TCEnv( const char* name )
        : Test::Case( name )
        {}

    virtual void Run()
        {
            const rbReal dtime = rbReal(1.0 / 60.0);
            const rbs32 div = 5;

            // 2秒後に y==0 となる高さ/加速度で落下させるテスト
            {
                rbEnvironment::Config config;
                config.RigidBodyCapacity = 20;
                config.ContactCapacty = 60;
                rbEnvironment env( config );

                const rbVec3 G( 0, rbReal(-10), 0 );
                rbRigidBody box;
                box.SetPosition( 0, 20, 0 );
                box.SetOrientation( 0, 0, 0 );

                env.Register( &box );

                for ( int i = 0; i < 120; ++i )
                {
                    box.SetForce( G );
                    env.Update( dtime, div );
                }

                TEST_ASSERT_DOUBLES_EQUAL( box.Position().y, rbReal(0), rbReal(0.05) );

                env.Unregister( &box );
            }

            // 箱同士の衝突(時刻はt==1.0付近、位置はx==0.0付近で衝突)
            {
                rbEnvironment::Config config;
                config.RigidBodyCapacity = 20;
                config.ContactCapacty = 60;
                rbEnvironment env( config );

                const rbVec3 G( 0, rbReal(-10), 0 );

                rbRigidBody box[2];

                box[0].SetPosition( rbReal(-10), 0, 0 );
                box[0].SetOrientation( 0, 0, 0 );
                box[0].SetLinearVelocity( rbReal(10), rbReal(10), 0 );
                env.Register( &box[0] );

                box[1].SetPosition( rbReal(10), 0, 0 );
                box[1].SetOrientation( 0, 0, 0 );
                box[1].SetLinearVelocity( rbReal(-10), rbReal(10), 0 );
                env.Register( &box[1] );

                rbReal t = rbReal(0);
                for ( int i = 0; i < 120; ++i )
                {
                    box[0].SetForce( G );
                    box[1].SetForce( G );
                    env.Update( dtime, div );

                    t += dtime;
                    // std::printf( "t=%f, box[0].Position() : (%f, %f, %f)\n",
                    //              t, box[0].Position().x, box[0].Position().y, box[0].Position().z );
                    // std::printf( "t=%f, box[1].Position() : (%f, %f, %f)\n",
                    //              t, box[1].Position().x, box[1].Position().y, box[1].Position().z );
                }

                env.Unregister( &box[0] );
                env.Unregister( &box[1] );
            }
        }
};

#endif
