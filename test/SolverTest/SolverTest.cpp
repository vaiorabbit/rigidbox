// -*- mode: C++; coding: utf-8 -*-
#include <TestFramework.h>

#include "TCSolver.h"

int
main( int argc, char** argv )
{
    Test::Suite suite( "Solver test" );

    Test::Case* tc[] = {
        new TCSolver( "Solver Test" ),
    };

    for ( int i = 0; i < sizeof(tc)/sizeof(tc[0]); ++i )
        suite.RegisterCase( tc[i] );

    suite.Run();

    if ( Test::ManagerInstance().FailCount() == 0 )
        std::cout << Test::ManagerInstance().AssertionCount() << " assertions succeeded." << std::endl;
    else
        std::cout << Test::ManagerInstance().FailCount() << " of " << Test::ManagerInstance().AssertionCount() << " assertions failed." << std::endl;

    for ( int i = 0; i < sizeof(tc)/sizeof(tc[0]); ++i )
        delete tc[i];

    return 0;
}
