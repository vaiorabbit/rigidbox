// -*- mode: C++; coding: utf-8; -*-
#ifndef TCSOLVER_H_INCLUDED
#define TCSOLVER_H_INCLUDED

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <RigidBox/RigidBox.h>
#include <TestFramework.h>

class TCSolver : public Test::Case
{
public:
    TCSolver( const char* name )
        : Test::Case( name )
        {}

    virtual void Run()
        {
            rbs32 result;
            const rbReal dt = rbReal(1.0 / 300.0);
            rbSolver solver( rbReal(0.2) );

            {
                // 立方体の辺同士で接するように配置
                rbContact c;
                rbRigidBody box0, box1;
                box0.SetPosition( rbReal(-1.41421), 0, 0 );
                box0.SetOrientation( 0, rbToRad(45), 0 );

                box1.SetPosition( rbReal(1.41421), 0, 0 );
                box1.SetOrientation( 0, rbToRad(45), 0 );

                result = rbCollision::Detect( &box0, &box1, &c );
                TEST_ASSERT( result == 1 );

                box0.UpdateVelocity( dt );
                box1.UpdateVelocity( dt );

                solver.ApplyImpulse( &c, dt );

                box0.CorrectVelocity();
                box1.CorrectVelocity();

                box0.UpdatePosition( dt );
                box1.UpdatePosition( dt );
            }

            {
                // 立方体の頂点同士が原点付近で接するように配置
                rbContact c;
                rbRigidBody box0, box1;
                box0.SetPosition( rbReal(-1.732), 0, 0 );
                box0.SetOrientation( 0, rbToRad(45), rbAsin(rbReal(1)/rbSqrt((3))) );
                box0.SetLinearVelocity( rbReal(1.0), 0, 0 );

                box1.SetPosition( rbReal(1.732), 0, 0 );
                box1.SetOrientation( 0, rbToRad(45), rbAsin(rbReal(1)/rbSqrt((3))) );
                box1.SetLinearVelocity( rbReal(-1.0), 0, 0 );

                result = rbCollision::Detect( &box0, &box1, &c );
                TEST_ASSERT( result == 1 );

                box0.UpdateVelocity( dt );
                box1.UpdateVelocity( dt );

                solver.ApplyImpulse( &c, dt );

                box0.CorrectVelocity();
                box1.CorrectVelocity();

                box0.UpdatePosition( dt );
                box1.UpdatePosition( dt );
            }
        }
};

#endif
