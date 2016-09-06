/*
 * example_references.cpp
 *
 * Author: Alex Barcelo
 */

#include <SimppleLogger.hpp>

void f1() {
    // This could be a logger for a group of functions,
    // in a whole file, dedicated to a class, etc.
    static pplelog::SimppleLogger *logger =
            pplelog::getLogger("myFunctionLogger");
    SPPLELOG_INFO(logger, "Attending call at f1");
    SPPLELOG_TRACE(logger, "still at f1");
}

void f2() {
    // Note that the name is the same for both f1() and f2()
    static pplelog::SimppleLogger *logger =
            pplelog::getLogger("myFunctionLogger");
    SPPLELOG_INFO(logger, "Attending call at f2");
    SPPLELOG_TRACE(logger, "still at f2");
}

int main() {
    // Reference to main application logger
    pplelog::SimppleLogger *logger = pplelog::getLogger("myAppLogger");

    SPPLELOG_INFO(logger, "Starting application");

    // Calling functions
    f1();
    f2();

    SPPLELOG_INFO(logger, "Changing level for the myFunctionLogger");
    pplelog::setLogLevel("myFunctionLogger", pplelog::TRACE);

    // Now both traces are shown
    f1();
    f2();

    // Note that both loggers are, in fact, the same.

    /*
     * Actual output:
     *
[myAppLogger]: INFO Starting application
[myFunctionLogger]: INFO Attending call at f1
[myFunctionLogger]: INFO Attending call at f2
[myAppLogger]: INFO Changing level for the myFunctionLogger
[myFunctionLogger]: INFO Attending call at f1
[myFunctionLogger]: TRACE still at f1
[myFunctionLogger]: INFO Attending call at f2
[myFunctionLogger]: TRACE still at f2
     *
     */
}
