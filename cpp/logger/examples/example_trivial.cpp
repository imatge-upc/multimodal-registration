/*
 * example_trivial.cpp
 *
 * Author: Alex Barcelo
 */

// Include the SimppleLogger header
#include <SimppleLogger.hpp>

int main() {
    // Define and initialize a reference to a logger called "myAppLogger"
    pplelog::SimppleLogger *logger = pplelog::getLogger("myAppLogger");

    SPPLELOG_TRACE(logger, "Messages on trace are not enabled by default");
    SPPLELOG_DEBUG(logger, "Default level is DEBUG");
    SPPLELOG_INFO(logger, "All higher levels are automatically seen");
    SPPLELOG_WARN(logger, "There are 6 levels");
    SPPLELOG_ERROR(logger, "Those are: ");
    SPPLELOG_ERROR(logger, "TRACE, DEBUG, INFO, WARN, ERROR and FATAL");
    SPPLELOG_FATAL(logger, "they are sorted this way");

    /*
     * Actual output:
     *
[myAppLogger]: DEBUG Default level is DEBUG
[myAppLogger]: INFO All higher levels are automatically seen
[myAppLogger]: WARN There are 6 levels
[myAppLogger]: ERROR Those are:
[myAppLogger]: ERROR TRACE, DEBUG, INFO, WARN, ERROR and FATAL
[myAppLogger]: FATAL they are sorted this way
     *
     */
}
