/*
 * example_callback.cpp
 *
 * Author: Alex Barcelo
 */

// Include the SimppleLogger header
#include <SimppleLogger.hpp>

void myLogCallback(std::string name, pplelog::levels level, std::string msg) {
	std::cout << "I (" << name << ") am working at level " << level << "("
	          << pplelog::getLevelDescription(level) << "). Message:"
	          << std::endl << msg << std::endl;
}

int main() {
    // Define and initialize a reference to a logger called "myAppLogger"
    pplelog::SimppleLogger *logger = pplelog::getLogger("myAppLogger");

    SPPLELOG_DEBUG(logger, "Default message in default callback");
    pplelog::setLogCallback(myLogCallback);
    SPPLELOG_DEBUG(logger, "My own callback");

#if __cplusplus >= 201103L
    pplelog::setLogCallback(
        [] (std::string name, pplelog::levels level, std::string msg) {
    		std::cout << "In a lambda function! I am " << name << std::endl
    				  << "msg:" << msg;
    });
    SPPLELOG_DEBUG(logger, "Testing lambda callback");

#endif


    /*
     * Actual output:
     *
[myAppLogger] |DEBUG|:Default message in default callback
I (myAppLogger) am working at level 0(DEBUG). Message:
My own callback
In a lambda function! I am myAppLogger
msg:Testing lambda callback
     *
     */
}
