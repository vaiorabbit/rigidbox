// -*- mode: C++; coding: utf-8; -*-
#ifndef RBSOLVER_H_INCLUDED
#define RBSOLVER_H_INCLUDED

#include "rbTypes.h"

class rbSolver
{
public:

    rbSolver( rbReal bias = rbReal(0.05) )
        : bias_factor( bias )
        {}

    void ApplyImpulse( rbContact* c, rbReal dt );

private:

    // Baumgarte 安定化法のパラメーター β に相当
    rbReal bias_factor;
};

#endif
