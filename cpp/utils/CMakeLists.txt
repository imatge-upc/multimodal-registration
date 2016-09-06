
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


find_package(Armadillo REQUIRED) # <<<
include_directories(${ARMADILLO_INCLUDE_DIRS}) # <<<


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(srcs
	src/edges.cpp
    )
SET(incs
    include/edges.h
    )
SET(impl_incs
	include/impl/edges.hpp
    )


add_library(utils SHARED ${srcs} ${incs} ${impl_incs})

target_link_libraries(utils ${ARMADILLO_LIBRARIES}) # <<<