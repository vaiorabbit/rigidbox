// -*- mode: C++; coding: utf-8; -*-
#ifndef TCINTEGRATION_H_INCLUDED
#define TCINTEGRATION_H_INCLUDED

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <RigidBox/RigidBox.h>
#include <TestFramework.h>

class TCIntegration : public Test::Case
{

public:
    TCIntegration( const char* name )
        : Test::Case( name )
        {}

    virtual void Run()
        {
            // 自由落下
            {
                // 2秒後に y==0 となる高さ/加速度で落下させるテスト
                rbRigidBody box;
                box.SetPosition( 0, 20, 0 );
                box.SetOrientation( 0, 0, 0 );
                box.UpdateInvInertiaWorld();

                const rbVec3 G( 0, rbReal(-10), 0 );
                const rbReal dt = rbReal(1.0 / 300.0);
                rbReal t = rbReal(0);

                for ( int i = 0; i < 600; ++i )
                {
                    box.SetForce( G );

                    box.UpdateVelocity( dt );
                    // box.ApplyImpulse(...);
                    box.CorrectVelocity();
                    box.UpdatePosition( dt );

                    box.ClearSolverWorkArea();
                    box.UpdateInvInertiaWorld();
                    box.SetForce( 0, 0, 0 );
                    box.SetTorque( 0, 0, 0 );

                    t += dt;
                    // std::printf( "t=%f, box.Position() : (%f, %f, %f)\n",
                    //              t, box.Position().x, box.Position().y, box.Position().z );
                }

                TEST_ASSERT_DOUBLES_EQUAL( box.Position().y, rbReal(0), rbReal(0.05) );
            }

            // 回転
            {
                // 2秒で一周する角速度で回すテスト
                // 回転は誤差が蓄積されやすく、ステップサイズを小さく取る必要がある
                rbRigidBody box;
                box.SetPosition( 0, 0, 0 );
                box.SetOrientation( 0, 0, 0 );
                box.UpdateInvInertiaWorld();

                const rbReal dt = rbReal(1.0 / 300.0);
                rbReal t = rbReal(0);

                box.SetAngularVelocity( 0, rbReal(RIGIDBOX_REAL_PI), 0 );
                for ( int i = 0; i < 600; ++i )
                {
                    box.UpdateVelocity( dt );
                    // box.ApplyImpulse(...);
                    box.CorrectVelocity();
                    box.UpdatePosition( dt );

                    box.ClearSolverWorkArea();
                    box.UpdateInvInertiaWorld();
                    box.SetForce( 0, 0, 0 );
                    box.SetTorque( 0, 0, 0 );

                    t += dt;
                    // std::printf( "t=%f, box.Orientation() : [(%f, %f, %f) (%f, %f, %f) (%f, %f, %f)]\n", t,
                    //              box.Orientation().r[0].x, box.Orientation().r[0].y, box.Orientation().r[0].z,
                    //              box.Orientation().r[1].x, box.Orientation().r[1].y, box.Orientation().r[1].z,
                    //              box.Orientation().r[2].x, box.Orientation().r[2].y, box.Orientation().r[2].z );
                    // std::printf("	%f\n", box.Orientation().Column(0).Length() );
                }

                rbReal tolerance(0.5);
                TEST_ASSERT_DOUBLES_EQUAL( box.Orientation().Column(0).x, rbReal(1), tolerance );
                TEST_ASSERT_DOUBLES_EQUAL( box.Orientation().Column(0).z, rbReal(0), tolerance );

                TEST_ASSERT_DOUBLES_EQUAL( box.Orientation().Column(2).x, rbReal(0), tolerance );
                TEST_ASSERT_DOUBLES_EQUAL( box.Orientation().Column(2).z, rbReal(1), tolerance );
            }

        }
};

#endif
