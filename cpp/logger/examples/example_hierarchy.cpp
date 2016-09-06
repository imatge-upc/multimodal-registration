/*
 * example_hierarchy.cpp
 *
 * Author: Alex Barcelo
 */

#include <SimppleLogger.hpp>

void f1() {
    // This could be a logger for a group of functions,
    // in a whole file, dedicated to a class, etc.
    static pplelog::SimppleLogger *logger =
            pplelog::getLogger("myF1Logger");
    SPPLELOG_INFO(logger, "Attending call at f1");
    SPPLELOG_TRACE(logger, "still at f1");
}

void f2() {
    // Note that the name is the same for both f1() and f2()
    static pplelog::SimppleLogger *logger =
            pplelog::getLogger("myF2Logger");
    SPPLELOG_INFO(logger, "Attending call at f2");
    SPPLELOG_TRACE(logger, "still at f2");
}

int main() {
    // Reference to main application logger
    pplelog::SimppleLogger *logger = pplelog::getLogger("myAppLogger");

    // Set hierarchy
    pplelog::setParent("myF1Logger", "functionLoggers");
    pplelog::setParent("myF2Logger", "functionLoggers");
    /* The logger functionLoggers exists implicitly,
     * from an organizative point of view.
     * Actually creating the instance and using it is up to you.
     */

    // Default levels
    f1();
    f2();

    SPPLELOG_DEBUG(logger, "Default level is debug");
    pplelog::setDefaultLevel(pplelog::INFO);
    // Changing the default level changes all non-explicit levels
    SPPLELOG_DEBUG(logger, "This is not displayed");

    // Changing the parent changes the behaviour of children
    SPPLELOG_INFO(logger, "Changing log level for functionLoggers");
    pplelog::setLogLevel("functionLoggers", pplelog::TRACE);
    f1();
    f2();

    // Note that setting a level explicitly overrides any parent change
    SPPLELOG_INFO(logger, "Setting explicit levels on children loggers");
    pplelog::setLogLevel("myF1Logger", pplelog::ALWAYS_LOG);
    pplelog::setLogLevel("myF2Logger", pplelog::FATAL);
    pplelog::setLogLevel("functionLoggers", pplelog::DEBUG);
    f1();
    f2();

    /*
     * Actual output:
     *
[myF1Logger]: INFO Attending call at f1
[myF2Logger]: INFO Attending call at f2
[myAppLogger]: DEBUG Default level is debug
[myAppLogger]: INFO Changing log level for functionLoggers
[myF1Logger]: INFO Attending call at f1
[myF1Logger]: TRACE still at f1
[myF2Logger]: INFO Attending call at f2
[myF2Logger]: TRACE still at f2
[myAppLogger]: INFO Setting explicit levels on children loggers
[myF1Logger]: INFO Attending call at f1
[myF1Logger]: TRACE still at f1
     *
     */

}
