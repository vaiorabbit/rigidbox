// -*- mode: C++; coding: utf-8; -*-
#include <RigidBox/rbCollision.h>
#include <RigidBox/rbMath.h>
#include <RigidBox/rbRigidBody.h>
#include <RigidBox/rbSolver.h>

void rbSolver::ApplyImpulse( rbContact* c, rbReal dt )
{
    rbVec3 relative_velocity =
         c->Body[0]->LinearVelocity() + (c->Body[0]->AngularVelocity() % c->RelativeBodyPosition[0])
      - (c->Body[1]->LinearVelocity() + (c->Body[1]->AngularVelocity() % c->RelativeBodyPosition[1]));

    rbReal impulse_magnitude;
    rbVec3 impulse;

    // [LANG en] Apply impulse generated from collision
    // [LANG ja] 衝突による速度変化を表すインパルスを剛体に適用
    {
        rbReal K[2] = {
            c->Body[0]->InvMass() + (c->Body[0]->InvInertiaWorld() * ((c->RelativeBodyPosition[0] % c->Normal) % c->RelativeBodyPosition[0])) * c->Normal,
            c->Body[1]->InvMass() + (c->Body[1]->InvInertiaWorld() * ((c->RelativeBodyPosition[1] % c->Normal) % c->RelativeBodyPosition[1])) * c->Normal,
        };
        rbReal e = c->Body[0]->Restitution() * c->Body[1]->Restitution();

        impulse_magnitude = -(1 + e) * (relative_velocity * c->Normal);
        impulse_magnitude += bias_factor * rbMax(c->PenetrationDepth-rbReal(0), 0) / dt;
        impulse_magnitude /= K[0] + K[1];
    }

    impulse = impulse_magnitude * c->Normal;
    c->Body[0]->ApplyImpulse(  impulse, c->RelativeBodyPosition[0] );
    c->Body[1]->ApplyImpulse( -impulse, c->RelativeBodyPosition[1] );

    // [LANG en] Calculate slowdown by friction as impulse
    // [LANG ja] 摩擦による減速もインパルスとして表現し剛体に適用
    rbVec3 tangent = c->Normal % (c->Normal % relative_velocity);
    {
        rbReal tangent_length = tangent.Length();
        if ( tangent_length > RIGIDBOX_TOLERANCE )
            tangent /= tangent_length;
        else
            tangent.SetZero();
    }

    {
        rbReal K[2] = {
            c->Body[0]->InvMass() + (c->Body[0]->InvInertiaWorld() * ((c->RelativeBodyPosition[0] % tangent) % c->RelativeBodyPosition[0])) * tangent,
            c->Body[1]->InvMass() + (c->Body[1]->InvInertiaWorld() * ((c->RelativeBodyPosition[1] % tangent) % c->RelativeBodyPosition[1])) * tangent,
        };

        if ( K[0] + K[1] > RIGIDBOX_TOLERANCE )
            impulse_magnitude = rbReal(1) / (K[0] + K[1]);
        else
            impulse_magnitude = rbReal(0);
    }

    rbReal coeff = rbMin( rbMax(rbFabs(tangent*relative_velocity), rbReal(0.0)), c->Body[0]->Friction()*c->Body[1]->Friction() );
    impulse = coeff * impulse_magnitude * tangent;
    c->Body[0]->ApplyImpulse(  impulse, c->RelativeBodyPosition[0] );
    c->Body[1]->ApplyImpulse( -impulse, c->RelativeBodyPosition[1] );
}

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
