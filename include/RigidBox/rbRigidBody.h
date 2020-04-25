// -*- mode: C++; coding: utf-8; -*-
#ifndef RBRIGIDBODY_H_INCLUDED
#define RBRIGIDBODY_H_INCLUDED

#include "rbTypes.h"
#include "rbMath.h"

class rbRigidBody
{
public:

    struct State
    {
        // [LANG en] +position+ represents 'center-of-mass position'.
        // [LANG ja] +position+ は重心の位置を示します。
        rbVec3 position;
        rbMtx3 orientation;

        rbVec3 linear_velocity;
        rbVec3 angular_velocity;

        rbVec3 force;
        rbVec3 torque;

        rbVec3 angular_momentum;
        rbMtx3 inv_inertia_world;

        State()
            : position(rbReal(0), rbReal(0), rbReal(0))
            , orientation(rbReal(1), 0, 0,
                          0, rbReal(1), 0,
                          0, 0, rbReal(1))
            , linear_velocity(rbReal(0), rbReal(0), rbReal(0))
            , angular_velocity(rbReal(0), rbReal(0), rbReal(0))
            , force(rbReal(0), rbReal(0), rbReal(0))
            , torque(rbReal(0), rbReal(0), rbReal(0))
            , angular_momentum(rbReal(0), rbReal(0), rbReal(0))
            , inv_inertia_world(rbReal(1), 0, 0,
                                0, rbReal(1), 0,
                                0, 0, rbReal(1))
            {}

    };

    struct SolverWorkArea
    {
        rbVec3 delta_linear_velocity;
        rbVec3 delta_angular_momentum;
        rbVec3 delta_angular_velocity;

        SolverWorkArea()
            : delta_linear_velocity(0, 0, 0)
            , delta_angular_momentum(0, 0, 0)
            , delta_angular_velocity(0, 0, 0)
            {}

        void Clear()
            {
                delta_linear_velocity.SetZero();
                delta_angular_momentum.SetZero();
                delta_angular_velocity.SetZero();
            }
    };

    struct Shape
    {
        // [LANG en] each elements correspond to the box's half-extent in x-, y- and z-direction.
        // [LANG ja] それぞれの要素が直方体の x-, y-, z- 軸方向の half-extent に対応します。
        rbVec3 half_extent;

        rbReal inv_mass;
        rbMtx3 inv_inertia;

        rbReal restitution_coefficient;
        rbReal friction_coefficient;

        Shape()
            : half_extent(rbReal(1), rbReal(1), rbReal(1))
            , inv_mass(rbReal(1))
            , inv_inertia(rbReal(1), 0, 0,
                          0, rbReal(1), 0,
                          0, 0, rbReal(1))
            , restitution_coefficient(rbReal(0.5))
            , friction_coefficient(rbReal(0.5))
            {}
    };

    struct SleepStatus
    {
        bool On;
        rbReal GoSleepThresholdLV;
        rbReal GoSleepThresholdAV;
        rbReal WakeUpThresholdLV;
        rbReal WakeUpThresholdAV;
        rbReal GoSleepDuration;
        rbReal SleepingDuration;

        SleepStatus()
            : On( false )
            , GoSleepThresholdLV( rbReal(0.03) )
            , GoSleepThresholdAV( rbReal(0.03) )
            , WakeUpThresholdLV( rbReal(1.0) )
            , WakeUpThresholdAV( rbReal(1.0) )
            , GoSleepDuration( rbReal(0.5) )
            , SleepingDuration( 0 )
            {}

        void Clear()
            {
                On = false;
                GoSleepThresholdLV = rbReal(0.03);
                GoSleepThresholdAV = rbReal(0.03);
                WakeUpThresholdLV = rbReal(1.0);
                WakeUpThresholdAV = rbReal(1.0);
                GoSleepDuration = rbReal(0.5);
                SleepingDuration = 0;
            }
    };

    static const rbu32 Attribute_None            = 0x00000000U;
    static const rbu32 Attribute_Fixed           = 0x00000001U;
    static const rbu32 Attribute_AutoSleep       = 0x00000002U;

    rbRigidBody()
        : state()
        , shape()
        , solver_work_area()
        , sleep_status()
        , attribute(Attribute_None)
        {}

    rbVec3 Position()
        { return state.position; }

    void SetPosition( rbReal x, rbReal y, rbReal z )
        { state.position.Set( x, y, z ); }

    void SetPosition( const rbVec3& v )
        { state.position = v; }

    void AddPosition( rbReal dx, rbReal dy, rbReal dz )
        { state.position.Add(dx, dy, dz); }

    void AddPosition( const rbVec3& dv )
        { state.position += dv; }


    rbMtx3 Orientation()
        { return state.orientation; }

    rbMtx3 OrientationTranspose()
        { return state.orientation.GetTransposed(); }

    void SetOrientation( const rbMtx3& m )
        { state.orientation = m; }

    void SetOrientation( rbReal rad_x, rbReal rad_y, rbReal rad_z );
    void AddOrientation( rbReal rad_dx, rbReal rad_dy, rbReal rad_dz );


    rbVec3 LinearVelocity()
        { return state.linear_velocity; }

    void SetLinearVelocity( rbReal x, rbReal y, rbReal z )
        { state.linear_velocity.Set( x, y, z ); }

    void SetLinearVelocity( const rbVec3& v )
        { state.linear_velocity = v; }

    void AddLinearVelocity( rbReal dx, rbReal dy, rbReal dz )
        { state.linear_velocity.Add(dx, dy, dz); }

    void AddLinearVelocity( const rbVec3& dv )
        { state.linear_velocity += dv; }


    rbVec3 AngularVelocity()
        { return state.angular_velocity; }

    void SetAngularVelocity( rbReal x, rbReal y, rbReal z );
    void SetAngularVelocity( const rbVec3& v );
    void AddAngularVelocity( rbReal dx, rbReal dy, rbReal dz );
    void AddAngularVelocity( const rbVec3& dv );


    rbVec3 AngularMomentum()
        { return state.angular_velocity; }

    void SetAngularMomentum( rbReal x, rbReal y, rbReal z )
        { state.angular_momentum.Set( x, y, z ); }

    void SetAngularMomentum( const rbVec3& v )
        { state.angular_momentum = v; }

    void AddAngularMomentum( rbReal dx, rbReal dy, rbReal dz )
        { state.angular_momentum.Add(dx, dy, dz); }

    void AddAngularMomentum( const rbVec3& dv )
        { state.angular_momentum += dv; }


    rbVec3 Force()
        { return state.force; }

    void SetForce( rbReal x, rbReal y, rbReal z )
        { state.force.Set( x, y, z ); }

    void SetForce( const rbVec3& v )
        { state.force = v; }

    void SetForceAt( const rbVec3& v, const rbVec3& at );

    void AddForce( rbReal dx, rbReal dy, rbReal dz )
        { state.force.Add(dx, dy, dz); }

    void AddForce( const rbVec3& dv )
        { state.force += dv; }

    void AddForceAt( const rbVec3& dv, const rbVec3& at );


    rbVec3 Torque()
        { return state.torque; }

    void SetTorque( rbReal x, rbReal y, rbReal z )
        { state.torque.Set( x, y, z ); }

    void SetTorque( const rbVec3& v )
        { state.torque = v; }

    void AddTorque( rbReal dx, rbReal dy, rbReal dz )
        { state.torque.Add(dx, dy, dz); }

    void AddTorque( const rbVec3& dv )
        { state.torque += dv; }


    void SetShapeParameter( rbReal mass,
                            rbReal hx, rbReal hy, rbReal hz,
                            rbReal restitution_coeff, rbReal friction_coeff );

    rbVec3 HalfExtent()
        { return shape.half_extent; }

    rbReal Restitution()
        { return shape.restitution_coefficient; }

    rbReal Friction()
        { return shape.friction_coefficient; }

    rbReal InvMass()
        { return shape.inv_mass; }

    rbMtx3 InvInertia()
        { return shape.inv_inertia; }

    rbMtx3 InvInertiaWorld()
        { return state.inv_inertia_world; }

    void UpdateInvInertiaWorld();


    rbu32 Attribute()
        { return attribute; }
    void EnableAttribute( rbu32 attr )
        { attribute |= attr; }
    void DisableAttribute( rbu32 attr )
        { attribute &= ~attr; }
    bool AttributeEnabled( rbu32 attr )
        { return (attribute & attr) != 0; }

    bool IsFixed()
        { return (attribute & Attribute_Fixed) != 0; }
    bool IsNotFixed()
        { return (attribute & Attribute_Fixed) == 0; }

    bool Sleeping()
        { return sleep_status.On == true; }
    bool Awake()
        { return sleep_status.On == false; }
    void SetSleepOn()
        { sleep_status.On = true; }
    void SetSleepOff()
        { sleep_status.On = false; }


    void UpdateVelocity( rbReal dt );
    void ApplyImpulse( const rbVec3& impulse, const rbVec3& relative_position );
    void CorrectVelocity();
    void UpdatePosition( rbReal dt );
    void UpdateSleepStatus( rbReal dt );

    void ClearSolverWorkArea()
        { solver_work_area.Clear(); }

    void ClearSleepStatus()
        { sleep_status.Clear(); }

    void ResetStatuses()
        {
            ClearSolverWorkArea();
            ClearSleepStatus();
        }

private:

    State state;
    Shape shape;
    SolverWorkArea solver_work_area;
    SleepStatus sleep_status;
    rbu32 attribute;
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

#endif
