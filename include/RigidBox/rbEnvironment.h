// -*- mode: C++; coding: utf-8; -*-
#ifndef RBENVIRONMENT_H_INCLUDED
#define RBENVIRONMENT_H_INCLUDED

#include <vector>
#include "rbSolver.h"
#include "rbTypes.h"

class rbEnvironment
{
public:

    typedef std::vector<rbRigidBody*> BodyPtrContainer;
    typedef std::vector<rbContact> ContactContainer;

    struct Config
    {
        rbs32 RigidBodyCapacity;
        rbs32 ContactCapacty;

        Config()
            : RigidBodyCapacity(10)
            , ContactCapacty(20)
            {}
    };

    rbEnvironment();
    rbEnvironment( const Config& config );
    ~rbEnvironment();


    rbRigidBody* RigidBody( rbu32 index )
        { return bodies.at( index ); }

    rbu32 RigidBodyCount()
        { return bodies.size(); }

    rbu32 RigidBodyCapacity()
        { return bodies.capacity(); }


    rbContact* Contact( rbu32 index )
        { return &contacts.at( index ); }

    rbu32 ContactCount()
        { return contacts.size(); }

    rbu32 ContactCapacity()
        { return contacts.capacity(); }


    bool Register( rbRigidBody* box );
    bool Unregister( rbRigidBody* box );

    void Update( rbReal dtime, int div );

private:

    BodyPtrContainer bodies;
    ContactContainer contacts;
    rbSolver solver;
    Config config;
};

#endif
