// -*- mode: C++; coding: utf-8; -*-
#ifndef RBTYPES_H_INCLUDED
#define RBTYPES_H_INCLUDED

#include <cfloat>
#include <cmath>


// #define RIGIDBOX_USE_DOUBLE_PRECISION


#if defined(RIGIDBOX_USE_DOUBLE_PRECISION)
typedef double rbReal;
#else
typedef float  rbReal;
#endif

#if defined(RIGIDBOX_USE_DOUBLE_PRECISION)
# define RIGIDBOX_TOLERANCE (1e-15)
# define RIGIDBOX_REAL_MAX DBL_MAX
#else
# define RIGIDBOX_TOLERANCE (1e-6f)
# define RIGIDBOX_REAL_MAX FLT_MAX
#endif

#define RIGIDBOX_REAL_PI rbReal(3.141592653589793)

#if defined(RIGIDBOX_USE_DOUBLE_PRECISION)

#define rbAcos(x)  acos(rbReal((x)))
#define rbAsin(x)  asin(rbReal((x)))
#define rbAtan(x)  atan(rbReal((x)))
#define rbAtan2(x) atan2(rbReal((x)))
#define rbCos(x)   cos(rbReal((x)))
#define rbSin(x)   sin(rbReal((x)))
#define rbTan(x)   tan(rbReal((x)))
#define rbSqrt(x)  sqrt(rbReal((x)))
#define rbFabs(x)  fabs(rbReal((x)))

#else

#define rbAcos(x)  acosf(rbReal((x)))
#define rbAsin(x)  asinf(rbReal((x)))
#define rbAtan(x)  atanf(rbReal((x)))
#define rbAtan2(x) atan2f(rbReal((x)))
#define rbCos(x)   cosf(rbReal((x)))
#define rbSin(x)   sinf(rbReal((x)))
#define rbTan(x)   tanf(rbReal((x)))
#define rbSqrt(x)  sqrtf(rbReal((x)))
#define rbFabs(x)  fabsf(rbReal((x)))

#endif

typedef char           rbs8;
typedef short          rbs16;
typedef int            rbs32;
typedef unsigned char  rbu8;
typedef unsigned short rbu16;
typedef unsigned int   rbu32;

struct rbMtx3;
struct rbVec3;

struct rbContact;
class rbCollision;
class rbEnvironment;
class rbRigidBody;
class rbSolver;

#endif
