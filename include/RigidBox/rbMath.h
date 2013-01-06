// -*- mode: C++; coding: utf-8; -*-
#ifndef RBMATH_H_INCLUDED
#define RBMATH_H_INCLUDED

#include "rbTypes.h"

inline rbReal rbToRad( rbReal deg )
{
    // (/ float-pi 180.0) -> 0.017453292519943295
    return rbReal(0.017453292519943295) * deg;
}

inline rbReal rbMin( rbReal a, rbReal b )
{
    return a >= b ? b : a;
}

inline rbReal rbMax( rbReal a, rbReal b )
{
    return a >= b ? a : b;
}

inline rbReal rbClamp( rbReal v, rbReal v_min, rbReal v_max )
{
    if ( v < v_min )
        return v_min;
    else if ( v > v_max )
        return v_max;
    else
        return v;
}

inline rbReal rbSign( rbReal v )
{
    return v <= -RIGIDBOX_TOLERANCE ? rbReal(-1) : rbReal(1);
}


//
// rbVec3 Declaration
//

struct rbVec3
{
    union
    {
        struct
        {
            rbReal x, y, z;
        };
        rbReal e[3];
    };

    rbVec3();
    rbVec3( rbReal x_, rbReal y_, rbReal z_ );
    rbVec3( rbReal* a );
    rbVec3( const rbVec3& other );
    rbVec3& operator =( const rbVec3& other );
    void Set( rbReal x_, rbReal y_, rbReal z_ );
    void SetZero();
    void Add( rbReal x_, rbReal y_, rbReal z_ );
    rbVec3 operator -() const;
    rbVec3 operator +( const rbVec3& v ) const;
    rbVec3& operator +=( const rbVec3& v );
    rbVec3 operator -( const rbVec3& v ) const;
    rbVec3& operator -=( const rbVec3& v );
    rbVec3 operator *( rbReal f ) const;
    rbVec3& operator *=( rbReal f );
    rbVec3 operator /( rbReal f ) const;
    rbVec3& operator /=( rbReal f );
    rbReal operator *( const rbVec3& v ) const;
    rbVec3 operator %( const rbVec3& v ) const;
    rbReal Length() const;
    rbReal LengthSq() const;
    rbVec3 GetNormalized() const;
    rbVec3& Normalize();

}; // struct rbVec3


//
// rbMtx3 Declaration
//

struct rbMtx3
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    rbVec3 c[3];
#else
    rbVec3 r[3];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)

    rbMtx3();
    rbMtx3( rbReal e00, rbReal e01, rbReal e02,
            rbReal e10, rbReal e11, rbReal e12,
            rbReal e20, rbReal e21, rbReal e22 );
    rbMtx3( const rbMtx3& other );
    rbMtx3& operator =( const rbMtx3& other );
    rbReal& Elem( rbs32 row, rbs32 col );
    const rbReal& Elem( rbs32 row, rbs32 col ) const;
    rbVec3 Row( rbs32 i ) const;
    rbVec3 Column( rbs32 i ) const;
    void Set( rbReal e00, rbReal e01, rbReal e02,
              rbReal e10, rbReal e11, rbReal e12,
              rbReal e20, rbReal e21, rbReal e22 );
    void SetZero();
    void SetIdentity();
    rbMtx3& SetFromAxisAngle( const rbVec3& normalized_axis, rbReal radian );
    void SetAsCrossProductMatrix( const rbVec3& v );
    void Orthonormalize();
    void Scale( rbReal f );
    rbMtx3 GetTransposed() const;
    rbMtx3& Transpose();
    rbMtx3 GetInverse() const;
    rbMtx3& Invert();
    rbMtx3 operator +( const rbMtx3& m ) const;
    rbMtx3& operator +=( const rbMtx3& m );
    rbMtx3 operator -( const rbMtx3& m ) const;
    rbMtx3& operator -=( const rbMtx3& m );
    rbMtx3 operator *( rbReal f ) const;
    rbMtx3& operator *=( rbReal f );
    rbMtx3 operator *( const rbMtx3& m ) const;
    rbMtx3& operator *=( const rbMtx3& m );
    rbVec3 operator *( const rbVec3& v ) const;

}; // struct rbMtx3


//
// rbVec3 Implementation
//

inline rbVec3::rbVec3()
{}

inline rbVec3::rbVec3( rbReal x_, rbReal y_, rbReal z_ )
    : x(x_), y(y_), z(z_)
{}

inline rbVec3::rbVec3( rbReal* a )
    : x(a[0]), y(a[1]), z(a[2])
{}

inline rbVec3::rbVec3( const rbVec3& other )
    : x(other.x), y(other.y), z(other.z)
{}

inline rbVec3& rbVec3::operator =( const rbVec3& other )
{
    x = other.x;  y = other.y;  z = other.z;

    return *this;
}

inline void rbVec3::Set( rbReal x_, rbReal y_, rbReal z_ )
{
    x = x_;  y = y_;  z = z_;
}

inline void rbVec3::SetZero()
{
    x = 0;  y = 0;  z = 0;
}

inline void rbVec3::Add( rbReal x_, rbReal y_, rbReal z_ )
{
    x += x_;  y += y_;  z += z_;
}

inline rbVec3 rbVec3::operator -() const
{
    return rbVec3( -x, -y, -z );
}

inline rbVec3 rbVec3::operator +( const rbVec3& v ) const
{
    return rbVec3( x+v.x, y+v.y, z+v.z );
}

inline rbVec3& rbVec3::operator +=( const rbVec3& v )
{
    x += v.x;  y += v.y;  z += v.z;
    return *this;
}

inline rbVec3 rbVec3::operator -( const rbVec3& v ) const
{
    return rbVec3( x-v.x, y-v.y, z-v.z );
}

inline rbVec3& rbVec3::operator -=( const rbVec3& v )
{
    x -= v.x;  y -= v.y;  z -= v.z;
    return *this;
}

inline rbVec3 rbVec3::operator *( rbReal f ) const
{
    return rbVec3( x*f, y*f, z*f );
}

inline rbVec3& rbVec3::operator *=( rbReal f )
{
    x *= f;  y *= f;  z *= f;
    return *this;
}

inline rbVec3 rbVec3::operator /( rbReal f ) const
{
    return rbVec3( x/f, y/f, z/f );
}

inline rbVec3& rbVec3::operator /=( rbReal f )
{
    x /= f;  y /= f;  z /= f;
    return *this;
}

inline rbReal rbVec3::operator *( const rbVec3& v ) const
{
    return x*v.x + y*v.y + z*v.z;
}

inline rbVec3 rbVec3::operator %( const rbVec3& v ) const
{
    return rbVec3( y*v.z - z*v.y,
                   z*v.x - x*v.z,
                   x*v.y - y*v.x );
}

inline rbReal rbVec3::Length() const
{
    return rbSqrt( x*x + y*y + z*z );
}

inline rbReal rbVec3::LengthSq() const
{
    return x*x + y*y + z*z;
}

inline rbVec3 rbVec3::GetNormalized() const
{
    rbReal l = Length();
    l = rbReal(1) / l;
    return rbVec3( l*x, l*y, l*z );
}

inline rbVec3& rbVec3::Normalize()
{
    rbReal l = Length();
    l = rbReal(1) / l;

    x *= l;
    y *= l;
    z *= l;

    return *this;
}


//
// rbMtx3 Implementation
//

inline rbMtx3::rbMtx3()
{}

inline rbMtx3::rbMtx3( rbReal e00, rbReal e01, rbReal e02,
                       rbReal e10, rbReal e11, rbReal e12,
                       rbReal e20, rbReal e21, rbReal e22 )
{
    Elem(0,0) = e00;  Elem(0,1) = e01;  Elem(0,2) = e02;
    Elem(1,0) = e10;  Elem(1,1) = e11;  Elem(1,2) = e12;
    Elem(2,0) = e20;  Elem(2,1) = e21;  Elem(2,2) = e22;
}

inline rbMtx3::rbMtx3( const rbMtx3& other )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0] = other.c[0];
    c[1] = other.c[1];
    c[2] = other.c[2];
#else
    r[0] = other.r[0];
    r[1] = other.r[1];
    r[2] = other.r[2];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline rbMtx3& rbMtx3::operator =( const rbMtx3& other )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0] = other.c[0];
    c[1] = other.c[1];
    c[2] = other.c[2];
#else
    r[0] = other.r[0];
    r[1] = other.r[1];
    r[2] = other.r[2];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return *this;
}

inline rbReal& rbMtx3::Elem( rbs32 row, rbs32 col )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return c[col].e[row];
#else
    return r[row].e[col];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline const rbReal& rbMtx3::Elem( rbs32 row, rbs32 col ) const
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return c[col].e[row];
#else
    return r[row].e[col];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline rbVec3 rbMtx3::Row( rbs32 i ) const
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return rbVec3( Elem(i,0), Elem(i,1), Elem(i,2) );
#else
    return r[i];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline rbVec3 rbMtx3::Column( rbs32 i ) const
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return c[i];
#else
    return rbVec3( Elem(0,i), Elem(1,i), Elem(2,i) );
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline void rbMtx3::Set( rbReal e00, rbReal e01, rbReal e02,
                         rbReal e10, rbReal e11, rbReal e12,
                         rbReal e20, rbReal e21, rbReal e22 )
{
    Elem(0,0) = e00;  Elem(0,1) = e01;  Elem(0,2) = e02;
    Elem(1,0) = e10;  Elem(1,1) = e11;  Elem(1,2) = e12;
    Elem(2,0) = e20;  Elem(2,1) = e21;  Elem(2,2) = e22;
}

inline void rbMtx3::SetZero()
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0].SetZero();
    c[1].SetZero();
    c[2].SetZero();
#else
    r[0].SetZero();
    r[1].SetZero();
    r[2].SetZero();
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline void rbMtx3::SetIdentity()
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0].Set( 1, 0, 0 );
    c[1].Set( 0, 1, 0 );
    c[2].Set( 0, 0, 1 );
#else
    r[0].Set( 1, 0, 0 );
    r[1].Set( 0, 1, 0 );
    r[2].Set( 0, 0, 1 );
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}

inline rbMtx3& rbMtx3::SetFromAxisAngle( const rbVec3& axis, rbReal radian )
{
    rbReal s = rbSin( radian );
    rbReal c = rbCos( radian );
    rbReal C = rbReal(1) - c;
    rbReal x = axis.x;
    rbReal y = axis.y;
    rbReal z = axis.z;

    Elem(0,0) = x*x*C + c;
    Elem(0,1) = x*y*C - z*s;
    Elem(0,2) = z*x*C + y*s;
    Elem(1,0) = x*y*C + z*s;
    Elem(1,1) = y*y*C + c;
    Elem(1,2) = y*z*C - x*s;
    Elem(2,0) = z*x*C - y*s;
    Elem(2,1) = y*z*C + x*s;
    Elem(2,2) = z*z*C + c;

    return *this;
}

inline void rbMtx3::SetAsCrossProductMatrix( const rbVec3& v )
{
    Set(    0, -v.z,  v.y,
          v.z,    0, -v.x,
         -v.y,  v.x,    0 );
}

//
// [LANG en] Orhotnomalization algorithm. Should only be used for the orientation matrix of rigid body (rbRigidBody::State::R).
// [LANG ja] 正規直交化(※注意：剛体の姿勢行列 rbRigidBody::State::R のみで利用すること)
//
// Ref.: Rigid Body Dynamics - Chris Hecker's Website
//       See the physics article series for Game Developer Magazine.
//       http://chrishecker.com/Rigid_Body_Dynamics
//
inline void rbMtx3::Orthonormalize()
{
    rbVec3 X( Column(0).Normalize() );
    rbVec3 Y( Column(1) );
    rbVec3 Z( (X % Y).Normalize() );
    Y = (Z % X).Normalize();

    Set( X.e[0], Y.e[0], Z.e[0],
         X.e[1], Y.e[1], Z.e[1],
         X.e[2], Y.e[2], Z.e[2] );
}

inline void rbMtx3::Scale( rbReal f )
{
    for ( rbs32 row = 0; row < 3; ++row )
        for ( rbs32 col = 0; col < 3; ++col )
            Elem(row, col) *= f;
}

inline rbMtx3 rbMtx3::GetTransposed() const
{
    return rbMtx3( Elem(0,0), Elem(1,0), Elem(2,0),
                   Elem(0,1), Elem(1,1), Elem(2,1),
                   Elem(0,2), Elem(1,2), Elem(2,2) );
}

inline rbMtx3& rbMtx3::Transpose()
{
    rbReal tmp;
    tmp = Elem(0,1); Elem(0,1) = Elem(1,0); Elem(1,0) = tmp;
    tmp = Elem(0,2); Elem(0,2) = Elem(2,0); Elem(2,0) = tmp;
    tmp = Elem(1,2); Elem(1,2) = Elem(2,1); Elem(2,1) = tmp;

    return *this;
}

inline rbMtx3 rbMtx3::GetInverse() const
{
#define E( row, col ) Elem((row), (col))
#define R( row, col ) result.Elem((row), (col))
#define DET( e00, e01, e10, e11 ) ((e00)*(e11)-(e01)*(e10))

    rbMtx3 result;
    R(0,0) =  DET( E(1,1),E(1,2), E(2,1),E(2,2) );
    R(0,1) = -DET( E(0,1),E(0,2), E(2,1),E(2,2) );
    R(0,2) =  DET( E(0,1),E(0,2), E(1,1),E(1,2) );

    R(1,0) = -DET( E(1,0),E(1,2), E(2,0),E(2,2) );
    R(1,1) =  DET( E(0,0),E(0,2), E(2,0),E(2,2) );
    R(1,2) = -DET( E(0,0),E(0,2), E(1,0),E(1,2) );

    R(2,0) =  DET( E(1,0),E(1,1), E(2,0),E(2,1) );
    R(2,1) = -DET( E(0,0),E(0,1), E(2,0),E(2,1) );
    R(2,2) =  DET( E(0,0),E(0,1), E(1,0),E(1,1) );

    rbReal det = E(0,0) * R(0,0) + E(0,1) * R(1,0) + E(0,2) * R(2,0);
    // assert( rmFabs(det) >= RIGIDBOX_TOLERANCE );

    result.Scale( 1 / det );

    return result;

#undef E
#undef R
#undef DET
}

inline rbMtx3& rbMtx3::Invert()
{
    *this = GetInverse();

    return *this;
}

inline rbMtx3 rbMtx3::operator +( const rbMtx3& m ) const
{
    return rbMtx3( Elem(0,0)+m.Elem(0,0), Elem(0,1)+m.Elem(0,1), Elem(0,2)+m.Elem(0,2),
                   Elem(1,0)+m.Elem(1,0), Elem(1,1)+m.Elem(1,1), Elem(1,2)+m.Elem(1,2),
                   Elem(2,0)+m.Elem(2,0), Elem(2,1)+m.Elem(2,1), Elem(2,2)+m.Elem(2,2) );
}

inline rbMtx3& rbMtx3::operator +=( const rbMtx3& m )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0] += m.c[0];
    c[1] += m.c[1];
    c[2] += m.c[2];
#else
    r[0] += m.r[0];
    r[1] += m.r[1];
    r[2] += m.r[2];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)

    return *this;
}

inline rbMtx3 rbMtx3::operator -( const rbMtx3& m ) const
{
    return rbMtx3( Elem(0,0)-m.Elem(0,0), Elem(0,1)-m.Elem(0,1), Elem(0,2)-m.Elem(0,2),
                   Elem(1,0)-m.Elem(1,0), Elem(1,1)-m.Elem(1,1), Elem(1,2)-m.Elem(1,2),
                   Elem(2,0)-m.Elem(2,0), Elem(2,1)-m.Elem(2,1), Elem(2,2)-m.Elem(2,2) );
}

inline rbMtx3& rbMtx3::operator -=( const rbMtx3& m )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0] -= m.c[0];
    c[1] -= m.c[1];
    c[2] -= m.c[2];
#else
    r[0] -= m.r[0];
    r[1] -= m.r[1];
    r[2] -= m.r[2];
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)

    return *this;
}

inline rbMtx3 rbMtx3::operator *( rbReal f ) const
{
    return rbMtx3( Elem(0,0)*f, Elem(0,1)*f, Elem(0,2)*f,
                   Elem(1,0)*f, Elem(1,1)*f, Elem(1,2)*f,
                   Elem(2,0)*f, Elem(2,1)*f, Elem(2,2)*f );
}

inline rbMtx3& rbMtx3::operator *=( rbReal f )
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    c[0] *= f;
    c[1] *= f;
    c[2] *= f;
#else
    r[0] *= f;
    r[1] *= f;
    r[2] *= f;
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)

    return *this;
}

inline rbMtx3 rbMtx3::operator *( const rbMtx3& m ) const
{
    rbMtx3 result;
    for ( rbs32 row = 0; row < 3; ++row )
        for ( rbs32 col = 0; col < 3; ++col )
            result.Elem(row, col) =
                Elem(row, 0) * m.Elem(0, col) +
                Elem(row, 1) * m.Elem(1, col) +
                Elem(row, 2) * m.Elem(2, col) ;

    return result;
}

inline rbMtx3& rbMtx3::operator *=( const rbMtx3& m )
{
    *this = *this * m;
    return *this;
}

inline rbVec3 rbMtx3::operator *( const rbVec3& v ) const
{
#if defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
    return rbVec3( Row(0) * v, Row(1) * v, Row(2) * v );
#else
    return rbVec3( r[0] * v, r[1] * v, r[2] * v );
#endif // defined(RIGIDBOX_USE_COLUMN_MAJOR_MATRIX)
}


//
// Binary operators
//

inline rbVec3 operator *( rbReal f, const rbVec3& v )
{
    return rbVec3( v.x*f, v.y*f, v.z*f );
}

inline rbMtx3 operator *( rbReal f, const rbMtx3& m )
{
    return m * f;
}

#endif
