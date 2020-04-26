// -*- mode: C++; coding: utf-8 -*-
#include <cstdlib>
#include <RigidBox/RigidBox.h>
#include <DemoViewer.h>

static inline rbReal frand()
{
    return rbReal(std::rand()) / rbReal(RAND_MAX);
}


class StackDemo : public Scene
{
    static const rbs32 BoxCount = 10;

    rbEnvironment* env;
    rbRigidBody box[BoxCount], floor;

public:

    StackDemo()
        {
            rbEnvironment::Config config;
            config.RigidBodyCapacity = 20;
            config.ContactCapacty = 100;
            env = new rbEnvironment( config );

            for ( rbs32 i = 0; i < BoxCount; ++i )
            {
                box[i].SetShapeParameter( 10.0f,
                                          1.0f, 1.0f, 1.0f,
                                          0.0f, 0.5f );
                box[i].EnableAttribute( rbRigidBody::Attribute_AutoSleep );
                env->Register( &box[i] );
            }

            floor.SetShapeParameter( 10000.0f,
                                     10.0f, 10.0f, 10.0f,
                                     0.1f, 0.3f );
            floor.EnableAttribute( rbRigidBody::Attribute_Fixed );
            env->Register( &floor );
        }

    virtual ~StackDemo()
        {
            env->Unregister( &floor );

            for ( rbs32 i = 0; i < BoxCount; ++i )
                env->Unregister( &box[i] );

            delete env;
        }

    virtual void Update( float dt )
        {
            const rbs32 div = 4; // 増やし過ぎると衝突点が多発し弾ける
            const rbVec3 G( 0, rbReal(-9.8), 0 );

            for ( rbs32 i = 0; i < BoxCount; ++i )
                box[i].SetForce( G );

            env->Update( dt, div );
        }

    virtual void Reset()
        {
            env->ClearContacts();
            for ( rbs32 i = 0; i < BoxCount; ++i )
            {
                box[i].ResetStatuses();
                box[i].SetAngularMomentum(0,0,0);

                box[i].SetPosition( frand()-rbReal(0.5), rbReal(2.5) + i * rbReal(4), frand()-rbReal(0.5) );
                box[i].SetOrientation( 0, rbToRad(rbReal(10*i)), 0 );
                box[i].SetLinearVelocity( 0, 0, 0 );
            }

            floor.ResetStatuses();
            floor.SetAngularMomentum( 0, 0, 0 );
            floor.SetPosition( 0, rbReal(-10), 0 );
            floor.SetOrientation( 0, 0, 0 );
        }

    virtual void Render( Renderer* renderer )
        {
            renderer->RenderEnvironment( env );
        }
};

int main( int argc, char* argv[] )
{
    // Application app;
    StackDemo demo;

    // app.Initialize( argc, argv );
    // app.RegisterScene( &demo );
    // app.Run();
    // app.Finalize();

    Application app;
    app.Initialize( argc, argv );
    app.RegisterScene( &demo );
    bool running = true;
    while(running) {
        running = app.MainLoop();
    }
    app.Finalize();

    return 0;
}
