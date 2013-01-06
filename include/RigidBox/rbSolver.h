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

    // [LANG en] corresponds to the Baumgarte stabilization parameter β.
    // [LANG ja] Baumgarte 安定化法のパラメーター β に相当します。
    rbReal bias_factor;
};

#endif
