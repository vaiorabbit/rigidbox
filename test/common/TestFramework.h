// -*- mode: C++; coding: utf-8; -*-
#ifndef TESTFRAMEWORK_H_INCLUDED
#define TESTFRAMEWORK_H_INCLUDED

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#define TEST_ASSERT( statement ) \
    Test::ManagerInstance().Assert( statement, #statement, __FILE__, __LINE__ )

#define TEST_ASSERT_EQUAL( statement1, statement2 ) \
    Test::ManagerInstance().AssertEqual( statement1, statement2, #statement1 " == " #statement2, __FILE__, __LINE__ )

#define TEST_ASSERT_DOUBLES_EQUAL( statement, actual, precision ) \
    Test::ManagerInstance().AssertEqualDouble( statement, actual, precision, "|" #statement " - " #actual "| < " #precision, __FILE__, __LINE__ )


namespace Test
{
    //
    // Test::Manager
    //
    class Manager
    {
    public:
        Manager()
            : assertion_count(0)
            , fail_count(0)
            {}

        void Reset()
            {
                assertion_count = 0;
                fail_count = 0;
            }

        int AssertionCount() { return assertion_count; }

        int FailCount() { return fail_count; }

        void Assert( bool statement, const char* message, const char* filename, int linenum )
            {
                if ( !statement )
                {
                    std::stringstream ss;
                    ss << "Assertion " << message << " failed (" << filename << " Line:" << linenum << ").";
                    std::cout << ss.str() << std::endl;

                    ++fail_count;
                }

                ++assertion_count;
            }

        template <typename T1, typename T2>
        void AssertEqual( T1 statement1, T2 statement2, const char* message, const char* filename, int linenum )
            {
                if ( statement1 != statement2 )
                {
                    std::stringstream ss;
                    ss << "Assertion " << message << " failed (" << filename << " Line:" << linenum << ").";
                    std::cout << ss.str() << std::endl;

                    ++fail_count;
                }

                ++assertion_count;
            }

        void AssertEqualDouble( double statement, double actual, double precision, const char* message, const char* filename, int linenum )
            {
                if ( std::fabs(statement - actual) > precision )
                {
                    std::stringstream ss;
                    ss << "Assertion " << message << " failed (" << filename << " Line:" << linenum << ").";
                    std::cout << ss.str() << std::endl;

                    ++fail_count;
                }

                ++assertion_count;
            }

    private:
        int assertion_count;
        int fail_count;
    };

    Manager& ManagerInstance()
    {
        static Manager manager;
        return manager;
    }


    //
    // Test::Case
    //
    class Case
    {
    public:
        Case( const char* name_ ) : name( name_ ) {}
        Case() : name(NULL) {}
        virtual ~Case() {};

        const char* Name() { return name; }

        virtual void Initialize() {};
        virtual void Run() {};
        virtual void Finalize() {};

    private:
        const char* name;
    };


    //
    // Test::Suite
    //
    class Suite
    {
    public:
        typedef std::vector<Case*> Cases;

        Suite( const char* name_ )
            : name( name_ )
            , test_cases()
            {}

        ~Suite() {}

        void RegisterCase( Case* c )
            {
                test_cases.push_back( c );
            }

        unsigned int CaseCount()
            {
                return test_cases.size();
            }

        void Run()
            {
                Cases::iterator test_cases_end = test_cases.end();
                for ( Cases::iterator current_case = test_cases.begin(); current_case != test_cases_end; ++current_case )
                {
                    Case* c = *current_case;
                    std::cout << "* Case \"" << c->Name() << "\" start." << std::endl;
                    c->Initialize();
                    c->Run();
                    c->Finalize();
                    std::cout << "* Case \"" << c->Name() << "\" done." << std::endl;
                }
            }

    private:
        const char* name;
        Cases test_cases;
    };
}

#endif
