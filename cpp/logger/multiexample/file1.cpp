#include <SimppleLogger.hpp>
#include "file1.hpp"

static pplelog::SimppleLogger *logger =
		pplelog::getLogger("myF1Logger");

void f1() {
    // This could be a logger for a group of functions,
    // in a whole file, dedicated to a class, etc.
    SPPLELOG_INFO(logger, "Attending call at f1");
    SPPLELOG_TRACE(logger, "still at f1");
}
