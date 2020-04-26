// -*- mode: C++; coding: utf-8 -*-
#include <RigidBox/RigidBox.h>
#include <DemoViewer.h>

class CollisionDemo : public Scene
{
    rbEnvironment env;
    rbRigidBody box[2], floor;

public:

    CollisionDemo()
        {
            box[0].SetShapeParameter( 10.0f,
                                      1.0f, 1.0f, 1.0f,
                                      0.0f, 0.5f );
            box[0].EnableAttribute( rbRigidBody::Attribute_AutoSleep );

            box[1].SetShapeParameter( 10.0f,
                                      1.0f, 1.0f, 1.0f,
                                      0.0f, 0.5f );
            box[1].EnableAttribute( rbRigidBody::Attribute_AutoSleep );

            env.Register( &box[0] );
            env.Register( &box[1] );

            floor.SetShapeParameter( 10000.0f,
                                     10.0f, 10.0f, 10.0f,
                                     0.1f, 0.3f );
            floor.EnableAttribute( rbRigidBody::Attribute_Fixed );
            env.Register( &floor );
        }

    virtual ~CollisionDemo()
        {
            env.Unregister( &floor );
            env.Unregister( &box[0] );
            env.Unregister( &box[1] );
        }

    virtual void Update( float dt )
        {
            const rbs32 div = 3;
            const rbVec3 G( 0, rbReal(-9.8), 0 );
            box[0].SetForce( G );
            box[1].SetForce( G );
            env.Update( dt, div );
        }

    virtual void Reset()
        {
            env.ClearContacts();
            box[0].ResetStatuses();
            box[1].ResetStatuses();
            box[0].SetAngularMomentum(0,0,0);
            box[1].SetAngularMomentum(0,0,0);

            box[0].SetPosition( rbReal(-10), rbSqrt(2), 0 );
            box[0].SetOrientation( rbToRad(45), rbToRad(45), 0 );
            box[0].SetLinearVelocity( rbReal(10), rbReal(10), 0 );

            box[1].SetPosition( rbReal(10), rbReal(1), 0 );
            box[1].SetOrientation( 0, 0, 0 );
            box[1].SetLinearVelocity( rbReal(-10), rbReal(10), 0 );

            floor.ResetStatuses();
            floor.SetAngularMomentum( 0, 0, 0 );
            floor.SetPosition( 0, rbReal(-10), 0 );
            floor.SetOrientation( 0, 0, 0 );
        }

    virtual void Render( Renderer* renderer )
        {
            renderer->RenderEnvironment( &env );
        }
};

int main( int argc, char* argv[] )
{
    Application app;
    CollisionDemo demo;

    app.Initialize( argc, argv );
    app.RegisterScene( &demo );
    bool running = true;
    while(running) {
        running = app.MainLoop();
    }
    app.Finalize();

    return 0;
}
