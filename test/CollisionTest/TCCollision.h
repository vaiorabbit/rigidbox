// -*- mode: C++; coding: utf-8; -*-
#ifndef TCCOLLISION_H_INCLUDED
#define TCCOLLISION_H_INCLUDED

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <RigidBox/RigidBox.h>
#include <TestFramework.h>

class TCCollision : public Test::Case
{
public:
    TCCollision( const char* name )
        : Test::Case( name )
        {}

    virtual void Run()
        {
            rbs32 result;

            {
                // 2個の立方体を完全に重ねた状態でテスト
                rbContact c;
                rbRigidBody box0, box1;
                result = rbCollision::Detect( &box0, &box1, &c );
                TEST_ASSERT( result == 1 );
            }

            {
                // 立方体の面同士が重なるように配置
                rbContact c;
                rbRigidBody box0, box1;
                box0.SetShapeParameter( rbReal(1),
                                        rbReal(1), rbReal(1), rbReal(1),
                                        rbReal(1), rbReal(0.5) );
                box0.SetPosition( rbReal(-1), 0, 0 );
                box0.UpdateInvInertiaWorld();

                box1.SetShapeParameter( rbReal(1),
                                        rbReal(1), rbReal(1), rbReal(1),
                                        rbReal(1), rbReal(0.5) );
                box1.SetPosition( rbReal(1-0.0001), 0, 0 );
                box1.UpdateInvInertiaWorld();

                result = rbCollision::Detect( &box0, &box1, &c );

                TEST_ASSERT( result == 1 );
            }

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
            }

            {
                // 立方体の頂点同士が原点付近で接するように配置
                rbContact c;
                rbRigidBody box0, box1;
                box0.SetPosition( rbReal(-1.732), 0, 0 );
                box0.SetOrientation( 0, rbToRad(45), rbAsin(rbReal(1)/rbSqrt((3))) );

                box1.SetPosition( rbReal(1.732), 0, 0 );
                box1.SetOrientation( 0, rbToRad(45), rbAsin(rbReal(1)/rbSqrt((3))) );

                result = rbCollision::Detect( &box0, &box1, &c );
                TEST_ASSERT( result == 1 );
            }
        }
};

#endif
