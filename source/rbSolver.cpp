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

    //
    // 衝突
    //
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

    //
    // 摩擦
    //
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
            impulse_magnitude = rbReal(1) / K[0] + K[1];
        else
            impulse_magnitude = rbReal(0);
    }

    rbReal coeff = rbMin( rbMax(rbFabs(tangent*relative_velocity), rbReal(0.0)), c->Body[0]->Friction()*c->Body[1]->Friction() );
    impulse = coeff * impulse_magnitude * tangent;
    c->Body[0]->ApplyImpulse(  impulse, c->RelativeBodyPosition[0] );
    c->Body[1]->ApplyImpulse( -impulse, c->RelativeBodyPosition[1] );
}
