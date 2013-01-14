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

#endif
