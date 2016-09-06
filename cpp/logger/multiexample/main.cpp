/*
 * example_hierarchy.cpp
 *
 * Author: Alex Barcelo
 */

#include <SimppleLogger.hpp>
#include "file1.hpp"

int main() {
    // Reference to main application logger
    pplelog::SimppleLogger *logger = pplelog::getLogger("myAppLogger");

    // Set hierarchy
    pplelog::setParent("myF1Logger", "functionLoggers");
    /* The logger functionLoggers exists implicitly,
     * from an organizative point of view.
     * Actually creating the instance and using it is up to you.
     */

    // Default levels
    f1();

    /*
     * Actual output:
     *
     */

}
