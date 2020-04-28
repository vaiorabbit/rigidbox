// -*- mode: C++; coding: utf-8; -*-
#include <RigidBox/rbRigidBody.h>

void rbRigidBody::SetOrientation( rbReal rad_x, rbReal rad_y, rbReal rad_z )
{
    // state.orientation =
    //     rbMtx3().SetFromAxisAngle(rbVec3(0,0,1), rad_z) *
    //     rbMtx3().SetFromAxisAngle(rbVec3(0,1,0), rad_y) *
    //     rbMtx3().SetFromAxisAngle(rbVec3(1,0,0), rad_x) ;

    state.orientation.SetFromAxisAngle( rbVec3(0,0,1), rad_z );

    rbMtx3 m;
    m.SetFromAxisAngle( rbVec3(0,1,0), rad_y );
    state.orientation *= m;

    m.SetFromAxisAngle( rbVec3(1,0,0), rad_x );
    state.orientation *= m;
}

void rbRigidBody::AddOrientation( rbReal rad_dx, rbReal rad_dy, rbReal rad_dz )
{
    rbMtx3 m, mAdd;

    mAdd.SetFromAxisAngle( rbVec3(0,0,1), rad_dz );

    m.SetFromAxisAngle( rbVec3(0,1,0), rad_dy );
    mAdd *= m;

    m.SetFromAxisAngle( rbVec3(1,0,0), rad_dx );
    mAdd *= m;

    state.orientation = mAdd * state.orientation;
}


void rbRigidBody::SetAngularVelocity( rbReal x, rbReal y, rbReal z )
{
    state.angular_velocity.Set( x, y, z );
    state.angular_momentum = state.inv_inertia_world * state.angular_velocity;
}

void rbRigidBody::SetAngularVelocity( const rbVec3& v )
{
    state.angular_velocity = v;
    state.angular_momentum = state.inv_inertia_world * state.angular_velocity;
}

void rbRigidBody::AddAngularVelocity( rbReal dx, rbReal dy, rbReal dz )
{
    state.angular_velocity += rbVec3(dx, dy, dz);
    state.angular_momentum = state.inv_inertia_world * state.angular_velocity;
}

void rbRigidBody::AddAngularVelocity( const rbVec3& dv )
{
    state.angular_velocity += dv;
    state.angular_momentum = state.inv_inertia_world * state.angular_velocity;
}


void rbRigidBody::SetForceAt( const rbVec3& v, const rbVec3& at )
{
    rbVec3 relative_position = at - state.position;
    state.torque = relative_position % v;
}

void rbRigidBody::AddForceAt( const rbVec3& dv, const rbVec3& at )
{
    rbVec3 relative_position = at - state.position;
    state.torque += relative_position % dv;
}


void rbRigidBody::SetShapeParameter(rbReal mass, rbReal hx, rbReal hy, rbReal hz, rbReal restitution_coeff, rbReal friction_coeff)
{
    shape.Set(mass, hx, hy, hz, restitution_coeff, friction_coeff);
}


void rbRigidBody::UpdateInvInertiaWorld()
{
    // I^-1 = R * I0^-1 * R^T
    state.inv_inertia_world = state.orientation * shape.inv_inertia * state.orientation.GetTransposed();
}

void rbRigidBody::UpdateVelocity( rbReal dt )
{
    if ( IsFixed() ) return;

    state.linear_velocity += shape.inv_mass * dt * state.force;

    state.angular_momentum += dt * state.torque;
    state.angular_velocity = state.inv_inertia_world * state.angular_momentum;
}

void rbRigidBody::ApplyImpulse( const rbVec3& impulse, const rbVec3& relative_position )
{
    if ( IsFixed() ) return;

    //
    // Ref.: Physics-Based Animation (2005) p.135, Theorem 6.2 (Applying Impulse to a Rigid Body)
    //
    // [LANG en] Take J as an impulse vector :
    // [LANG ja] 力積を J とおくと：
    // - Δv = J / m
    // - Δω = I^-1 * (r × J)
    //

    solver_work_area.delta_linear_velocity += shape.inv_mass * impulse;

    rbVec3 L = relative_position % impulse;
    solver_work_area.delta_angular_momentum += L;
    solver_work_area.delta_angular_velocity += state.inv_inertia_world * L;
}

void rbRigidBody::CorrectVelocity()
{
    if ( IsFixed() ) return;

    state.linear_velocity += solver_work_area.delta_linear_velocity;

    state.angular_momentum += solver_work_area.delta_angular_momentum;
    state.angular_velocity += solver_work_area.delta_angular_velocity;
}

void rbRigidBody::UpdatePosition( rbReal dt )
{
    if ( IsFixed() ) return;

    state.position += dt * state.linear_velocity;

    rbMtx3 rot;
    rot.SetAsCrossProductMatrix( state.angular_velocity );
    state.orientation += dt * rot * state.orientation;

    state.orientation.Orthonormalize();
}

void rbRigidBody::UpdateSleepStatus( rbReal dt )
{
    if ( !AttributeEnabled(Attribute_AutoSleep) )
        return;

    rbReal thresholdLV = state.linear_velocity.Length();
    rbReal thresholdAV = state.angular_velocity.Length();

    if ( !sleep_status.On && thresholdLV < sleep_status.GoSleepThresholdLV && thresholdAV < sleep_status.GoSleepThresholdAV )
    {
        sleep_status.SleepingDuration += dt;
        if ( sleep_status.SleepingDuration > sleep_status.GoSleepDuration )
            sleep_status.On = true;
    }
    else if ( sleep_status.On &&
              (thresholdLV > sleep_status.WakeUpThresholdLV || thresholdAV > sleep_status.WakeUpThresholdAV) )
    {
        sleep_status.SleepingDuration = 0;
        sleep_status.On = false;
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
