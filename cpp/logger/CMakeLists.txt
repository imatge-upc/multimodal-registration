
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
SET(srcs
    src/SimppleLogger.cpp
    )
SET(incs
    src/SimppleLogger.hpp
    )
SET(impl_incs
    )

add_library(logger SHARED ${srcs} ${incs} ${impl_incs})
