// -*- mode: C++; coding: utf-8 -*-
#include <RigidBox/RigidBox.h>
#include <DemoViewer.h>

class DominoDemo : public Scene
{
    rbEnvironment env;
    rbRigidBody box[5], floor;

public:

    DominoDemo()
    {
        box[0].SetShapeParameter(
            10.0f,
            0.5f, 0.5f, 0.5f,
            0.5f, 0.5f);

        box[1].SetShapeParameter(100.0f,
            0.25f, 1.5f, 1.0f,
            0.0f, 0.5f);

        box[2].SetShapeParameter(125.0f,
            0.30f, 2.0f, 1.0f,
            0.0f, 0.5f);

        box[3].SetShapeParameter(150.0f,
            0.35f, 2.5f, 1.0f,
            0.0f, 0.5f);

        box[4].SetShapeParameter(150.0f,
            0.40f, 3.0f, 1.0f,
            0.0f, 0.5f);

        for (auto& b : box) {
            b.EnableAttribute(rbRigidBody::Attribute_AutoSleep);
            env.Register(&b);
        }
        floor.SetShapeParameter(
            10000.0f,
            10.0f, 10.0f, 10.0f,
            0.0f, 0.8f);
        floor.EnableAttribute(rbRigidBody::Attribute_Fixed);
        env.Register(&floor);
    }

    virtual ~DominoDemo()
    {
        env.Unregister(&floor);
        for (auto& b : box) {
            env.Unregister(&b);
        }
    }

    virtual void Update( float dt )
        {
            const rbs32 div = 1;
            const rbVec3 G( 0, 10*rbReal(-9.8), 0 );
            for (auto& b : box) {
                b.SetForce(G);
            }
            env.Update( dt, div );
        }

    virtual void Reset()
    {
        env.ClearContacts();
        for (auto& b : box) {
            b.ResetStatuses();
            b.SetAngularMomentum(0, 0, 0);
        }

        box[0].SetPosition(rbReal(-10), rbSqrt(2), 0);
        box[0].SetOrientation(rbToRad(45), rbToRad(45), 0);
        box[0].SetLinearVelocity(rbReal(50), rbReal(2.5), 0);

        box[1].SetPosition(rbReal(0), rbReal(1.5), 0);
        box[1].SetOrientation(0, 0, 0);
        box[1].SetLinearVelocity(rbReal(0), rbReal(0), 0);

        box[2].SetPosition(rbReal(3), rbReal(2), 0);
        box[2].SetOrientation(0, 0, 0);
        box[2].SetLinearVelocity(rbReal(0), rbReal(0), 0);

        box[3].SetPosition(rbReal(6), rbReal(2.5), 0);
        box[3].SetOrientation(0, 0, 0);
        box[3].SetLinearVelocity(rbReal(0), rbReal(0), 0);

        box[4].SetPosition(rbReal(9), rbReal(3.0), 0);
        box[4].SetOrientation(0, 0, 0);
        box[4].SetLinearVelocity(rbReal(0), rbReal(0), 0);

        floor.ResetStatuses();
        floor.SetAngularMomentum(0, 0, 0);
        floor.SetPosition(0, rbReal(-10), 0);
        floor.SetOrientation(0, 0, 0);
    }

    virtual void Render(Renderer* renderer)
    {
        renderer->RenderEnvironment(&env);
    }
};

int main( int argc, char* argv[] )
{
    Application app;
    DominoDemo demo;

    app.Initialize( argc, argv );
    app.RegisterScene( &demo );
    bool running = true;
    while(running) {
        running = app.MainLoop();
    }
    app.Finalize();

    return 0;
}
