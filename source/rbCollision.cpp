// -*- mode: C++; coding: utf-8; -*-
#include <RigidBox/rbRigidBody.h>
#include <RigidBox/rbCollision.h>

static inline rbReal HalfExtentOnAxis( const rbVec3& axis, const rbVec3& h, const rbMtx3& RT )
{
    rbVec3 axis_boxlocal = RT * axis;
    return
        rbFabs(axis_boxlocal.x) * h.x +
        rbFabs(axis_boxlocal.y) * h.y +
        rbFabs(axis_boxlocal.z) * h.z ;
}

static inline rbReal OverlapAlongAxis( const rbVec3& axis, rbVec3 h[2], rbMtx3 RT[2], const rbVec3& distance )
{
    rbReal r0 = HalfExtentOnAxis( axis, h[0], RT[0] );
    rbReal r1 = HalfExtentOnAxis( axis, h[1], RT[1] );
    rbReal D  = rbFabs( axis * distance );

    return r0 + r1 - D;
}

static inline rbReal Sign( rbReal v )
{
    return v < -RIGIDBOX_TOLERANCE ? rbReal(-1) : rbReal(1);
}

// [LANG en] [NOTE] In the context of GJK algorithm, this function is 'support map function', that returns 'support point' of a box.
// [LANG ja] [NOTE] この関数は GJK アルゴリズムでいうところのサポート写像で、出力として立方体の支点 (support point) を返すもの。
// Ref.: Gino van den Bergen, Collision Detection in Interactive 3D Enviroments, 4.3.4 Support Mappings (pp.130-139)
static inline rbVec3 FurthestVertexAlongAxis( const rbVec3& axis, const rbVec3& h, const rbMtx3& R, const rbMtx3& RT, const rbVec3& P )
{
    rbVec3 axis_boxlocal = (RT * axis).Normalize();

    rbVec3 FurthestVertexLocal( Sign(axis_boxlocal.x) * h.x, Sign(axis_boxlocal.y) * h.y, Sign(axis_boxlocal.z) * h.z );

    return R * FurthestVertexLocal + P;
}

static inline rbReal Clamp( rbReal val, rbReal min, rbReal max )
{
    if ( val < min )
        val = min;
    else if ( val > max )
        val = max;

    return val;
}

// Ref.: Christer Ericson, Real-Time Collision Detection (2005)
// 5.1.9 Closest Points of Two Line Segments
static inline void ClosestPointOfSegments( const rbVec3 colliding_edge[2][2], rbVec3 point_out[2] )
{
    rbVec3 d[2] = {
        colliding_edge[0][1] - colliding_edge[0][0],
        colliding_edge[1][1] - colliding_edge[1][0]
    };

    rbVec3 r = colliding_edge[0][0] - colliding_edge[1][0];

    rbReal a = d[0] * d[0];
    rbReal e = d[1] * d[1];
    rbReal c = d[0] * r;
    rbReal f = d[1] * r;
    rbReal b = d[0] * d[1];

    // [LANG en] If you want to make this routine more generic, you must confirm given edges don't degenerate into a point (given edges are not zero-length).
    // [LANG en] But the argument +colliding_edge+ is always given with valid edge data, we don't have to check the condition.
    // [LANG ja] より汎用的な交差判定ルーチンにしたいのであれば、ここで「辺の両端として渡された2点が一致していないかどうか
    // [LANG ja] (==点に縮退していないかどうか)」をチェックする必要がある。しかしここで colliding_edge にその可能性はないのでチェックは不要。

    rbReal t[2];
    rbReal denom = a*e - b*b;
    if ( denom > RIGIDBOX_TOLERANCE )
        t[0] = Clamp( (b*f - c*e) / denom, rbReal(0), rbReal(1) );
    else
        t[0] = rbReal(0);

    t[1] = (b * t[0] + f) / e;

    if ( t[1] < rbReal(0) )
    {
        t[1] = rbReal(0);
        t[0] = Clamp( -c / a, rbReal(0), rbReal(1) );
    }
    else if ( t[1] > rbReal(1) )
    {
        t[1] = rbReal(1);
        t[0] = Clamp( (b - c) / a, rbReal(0), rbReal(1) );
    }

    point_out[0] = colliding_edge[0][0] + d[0] * t[0];
    point_out[1] = colliding_edge[1][0] + d[1] * t[1];
}


// Separating axis identifier
typedef enum {
    SeparatingAxis_Box0XxBox1X = 0,
    SeparatingAxis_Box0XxBox1Y,
    SeparatingAxis_Box0XxBox1Z,
    SeparatingAxis_Box0YxBox1X,
    SeparatingAxis_Box0YxBox1Y,
    SeparatingAxis_Box0YxBox1Z,
    SeparatingAxis_Box0ZxBox1X,
    SeparatingAxis_Box0ZxBox1Y,
    SeparatingAxis_Box0ZxBox1Z,

    SeparatingAxis_Box0X,
    SeparatingAxis_Box0Y,
    SeparatingAxis_Box0Z,
    SeparatingAxis_Box1X,
    SeparatingAxis_Box1Y,
    SeparatingAxis_Box1Z,
} SeparatingAxis;

static const rbs32 ColumnIndices[9][2] = {
    { 0, 0 }, // SeparatingAxis_Box0XxBox1X (== 0) -> box0->Orientation().Column(0) % box1->Orientation().Column(0)
    { 0, 1 }, // SeparatingAxis_Box0XxBox1Y (== 1) -> box0->Orientation().Column(0) % box1->Orientation().Column(1)
    { 0, 2 }, // SeparatingAxis_Box0XxBox1Z (== 2) -> :
    { 1, 0 }, // SeparatingAxis_Box0YxBox1X (== 3)
    { 1, 1 }, // SeparatingAxis_Box0YxBox1Y (== 4)
    { 1, 2 }, // SeparatingAxis_Box0YxBox1Z (== 5)
    { 2, 0 }, // SeparatingAxis_Box0ZxBox1X (== 6)
    { 2, 1 }, // SeparatingAxis_Box0ZxBox1Y (== 7) -> :
    { 2, 2 }, // SeparatingAxis_Box0ZxBox1Z (== 8) -> box0->Orientation().Column(2) % box1->Orientation().Column(2)
};

//
// [LANG en] Box-Box collision detection algorithm
// [LANG ja] 立方体同士の衝突検出
//
// Ref.:
// - Open Dynamics Engine [box.cpp]
// - Game Physics Engine Development [collide_fine.cpp]
//

#define SATx( enAxis ) \
       current_axis = R[0].Column(ColumnIndices[enAxis][0]) % R[1].Column(ColumnIndices[enAxis][1]); \
       if ( current_axis.LengthSq() >= RIGIDBOX_TOLERANCE )                                          \
       {                                                                                             \
           current_axis.Normalize();                                                                 \
           current_penetration = OverlapAlongAxis( current_axis, h, RT, distance );                  \
           if ( current_penetration <= RIGIDBOX_TOLERANCE )                                          \
               return 0;                                                                             \
           if ( current_penetration < best_penetration )                                             \
           {                                                                                         \
               best_penetration = current_penetration;                                               \
               best_axis = current_axis;                                                             \
               best_axis_id = enAxis;                                                                \
           }                                                                                         \
       }

#define SAT0( enAxis ) \
       current_axis = R[0].Column(enAxis-SeparatingAxis_Box0X);                 \
       current_penetration = OverlapAlongAxis( current_axis, h, RT, distance ); \
       if ( current_penetration <= RIGIDBOX_TOLERANCE )                         \
           return 0;                                                            \
       if ( current_penetration < best_penetration )                            \
       {                                                                        \
           best_penetration = current_penetration;                              \
           best_axis = current_axis;                                            \
           best_axis_id = enAxis;                                               \
       }

#define SAT1( enAxis ) \
       current_axis = R[1].Column(enAxis-SeparatingAxis_Box1X);                 \
       current_penetration = OverlapAlongAxis( current_axis, h, RT, distance ); \
       if ( current_penetration <= RIGIDBOX_TOLERANCE )                         \
           return 0;                                                            \
       if ( current_penetration < best_penetration )                            \
       {                                                                        \
           best_penetration = current_penetration;                              \
           best_axis = current_axis;                                            \
           best_axis_id = enAxis;                                               \
       }


rbs32 rbCollision::Detect( rbRigidBody* box0, rbRigidBody* box1, rbContact* contact_out )
{
    rbVec3 current_axis;
    rbVec3 best_axis;
    SeparatingAxis best_axis_id;
    rbReal current_penetration;
    // [LANG en] Current candidate value of penetration depth along +best_axis_id+.
    // [LANG ja] 軸 +best_axis_id+ に沿った貫通深度の候補値
    rbReal best_penetration = RIGIDBOX_REAL_MAX;

    rbVec3 h[2] = { box0->HalfExtent(), box1->HalfExtent() };
    rbMtx3 R[2] = { box0->Orientation(), box1->Orientation() };
    rbMtx3 RT[2] = { box0->OrientationTranspose(), box1->OrientationTranspose() };
    rbVec3 P[2] = { box0->Position(), box1->Position() };
    rbVec3 distance = P[1] - P[0];

    //
    // [LANG en] Separating-Axis Test
    // [LANG ja] 分離軸テスト
    //

    // [LANG en] SAT using local axes of Box0
    // [LANG ja] Box0 のローカル座標系の軸を利用した分離軸テスト
    SAT0( SeparatingAxis_Box0X );
    SAT0( SeparatingAxis_Box0Y );
    SAT0( SeparatingAxis_Box0Z );

    // [LANG en] SAT using local axes of Box1
    // [LANG ja] Box1 のローカル座標系の軸を利用した分離軸テスト
    SAT1( SeparatingAxis_Box1X );
    SAT1( SeparatingAxis_Box1Y );
    SAT1( SeparatingAxis_Box1Z );

    // [LANG en] SAT using cross product from the local axes of each boxes
    // [LANG ja] Box0 ・ Box1 それぞれのローカル座標系の軸から作成した分離軸でのテスト
    SATx( SeparatingAxis_Box0XxBox1X );
    SATx( SeparatingAxis_Box0XxBox1Y );
    SATx( SeparatingAxis_Box0XxBox1Z );
    SATx( SeparatingAxis_Box0YxBox1X );
    SATx( SeparatingAxis_Box0YxBox1Y );
    SATx( SeparatingAxis_Box0YxBox1Z );
    SATx( SeparatingAxis_Box0ZxBox1X );
    SATx( SeparatingAxis_Box0ZxBox1Y );
    SATx( SeparatingAxis_Box0ZxBox1Z );

    //
    // [LANG en] No gap is found along any separating axes -> boxes are intersecting
    // [LANG ja] どの軸上でも隙間が確認できない⇒交差している
    //

    switch ( best_axis_id )
    {

    // [LANG en] a vertex of Box1 is touching the face of Box0
    // [LANG ja] Box1 の頂点が Box0 の面と交差している場合
    case SeparatingAxis_Box0X:
    case SeparatingAxis_Box0Y:
    case SeparatingAxis_Box0Z:
    {
        contact_out->Normal = best_axis;
        // [LANG en] By convention, +Normal+ should point from Box1 to Box0
        // [LANG ja] Box1 -> Box0 と向くように調整
        if ( distance.Normalize() * best_axis >= 0 )
            contact_out->Normal *= -1;

        contact_out->Position = FurthestVertexAlongAxis( contact_out->Normal, h[1], R[1], RT[1], P[1] );
    }
    break;

    // [LANG en] a vertex of Box0 is touching the face of Box1
    // [LANG ja] Box0 の頂点が Box1 の面と交差している場合
    case SeparatingAxis_Box1X:
    case SeparatingAxis_Box1Y:
    case SeparatingAxis_Box1Z:
    {
        contact_out->Normal = best_axis;
        // [LANG en] By convention, +Normal+ should point from Box1 to Box0
        // [LANG ja] Box1 -> Box0 と向くように調整
        if ( distance.Normalize() * best_axis >= 0 )
            contact_out->Normal *= -1;

        contact_out->Position = FurthestVertexAlongAxis( -contact_out->Normal, h[0], R[0], RT[0], P[0] );
    }
    break;

    // [LANG en] Both boxes are touching with each other's edge
    // [LANG ja] Box0 と Box1 の辺同士が交差している場合
    default:
    {
        // [LANG en] Create Normal
        // [LANG ja] 法線の決定
        contact_out->Normal = best_axis;
        // [LANG en] By convention, +Normal+ should point from Box1 to Box0
        // [LANG ja] Box1 -> Box0 と向くように調整
        if ( distance.Normalize() * best_axis >= 0 )
        {
            contact_out->Normal *= -1;
            best_axis *= -1;
        }

        // [LANG en] Estimate touching position
        // [LANG en] 1. identify touching edges
        // [LANG en] 2. calculate closest points on these edges
        // [LANG en] 3. take the midpoint of the points as the touching position
        // [LANG ja] あとはひたすら接触点の位置の推定
        // [LANG ja] 1. 接触状態にある辺2本を特定
        // [LANG ja] 2. 辺対辺の最近接点2個を求める
        // [LANG ja] 3. 上記の2個の点の中点を接触点の位置とする

        const rbs32 *ColIdx = ColumnIndices[best_axis_id];

        const rbVec3 best_axis_boxlocal[2] = {
            RT[0] * best_axis,
            RT[1] * best_axis
        };

        rbVec3 midpoint_on_colliding_edge[2] = {
            rbVec3(0, 0, 0),
            rbVec3(0, 0, 0)
        };

        for ( int i = 0; i < 3; ++i )
        {
            if ( i != ColIdx[0] )
                midpoint_on_colliding_edge[0].e[i] =
                    best_axis_boxlocal[0].e[i] < 0 ? h[0].e[i] : -h[0].e[i];

            if ( i != ColIdx[1] )
                midpoint_on_colliding_edge[1].e[i] =
                    best_axis_boxlocal[1].e[i] > 0 ? h[1].e[i] : -h[1].e[i];
        }

        // [LANG en] convert to the world coordinate system
        // [LANG ja] ワールド座標系での位置へ変換
        midpoint_on_colliding_edge[0] = R[0] * midpoint_on_colliding_edge[0] + P[0];
        midpoint_on_colliding_edge[1] = R[1] * midpoint_on_colliding_edge[1] + P[1];

        // [LANG en] The end points of a colliding edge can be found at positions h (and -h) away from the midpoint.
        // [LANG ja] 中点がわかればそこから軸方向に幅 h (および -h) だけ伸ばした位置が衝突している辺を表す両端となる
        const rbVec3 colliding_edge[2][2] = {
            {
                midpoint_on_colliding_edge[0] + h[0].e[ColIdx[0]] * R[0].Column(ColIdx[0]),
                midpoint_on_colliding_edge[0] - h[0].e[ColIdx[0]] * R[0].Column(ColIdx[0]),
            },
            {
                midpoint_on_colliding_edge[1] + h[1].e[ColIdx[1]] * R[1].Column(ColIdx[1]),
                midpoint_on_colliding_edge[1] - h[1].e[ColIdx[1]] * R[1].Column(ColIdx[1]),
            },
        };

        rbVec3 point_out[2];
        ClosestPointOfSegments( colliding_edge, point_out );

        // [LANG en] Tweak contact_out->Position
        // [LANG en] - take the average of point_out[0] and point_out[1], and halve +best_penetration+, or
        // [LANG ja] contact_out->Position の調整
        // [LANG ja] - point_out[0] と point_out[1] の平均にして best_penetration を 1/2 とする
        contact_out->Position = rbReal(0.5) * (point_out[0] + point_out[1]);
        best_penetration *= rbReal(0.5);
    }
    break;
    }

    // [LANG en] Collect type-independent (vertex-face or edge-edge) data
    // [LANG ja] 接触のタイプ(頂点対面 or 辺対辺)に依存しない情報はここで整理
    contact_out->RelativeBodyPosition[0] = contact_out->Position - P[0];
    contact_out->RelativeBodyPosition[1] = contact_out->Position - P[1];
    contact_out->Body[0] = box0;
    contact_out->Body[1] = box1;
    contact_out->PenetrationDepth = best_penetration;

    return 1;
}
