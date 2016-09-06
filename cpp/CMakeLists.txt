cmake_minimum_required(VERSION 2.8)
project(PhD)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE) # setting Debug configuration

# Binaries to produce
set(TOOLS 
    canny
    evalDetection
    featureMatching-multimodal
    featureMatching-multimodal-eval
    flatten_cloud
    gradientEdgePointCloud-multiKcolor
    hystheresisThresholding
    image2cloud
    noise
    pcdViewer
    removenan
)

# Configure PCL (copied form pointclouds.org, and added custom PCL location)
set(PCL_DIR "${CMAKE_SOURCE_DIR}/../pcl-build")
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Configure OpenCV
set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/../opencv-build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Configure Boost WARNING! PCL will break the configuration if done before!
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Adding utils custom library
add_subdirectory (utils)
include_directories("${CMAKE_SOURCE_DIR}/utils/include")

# Adding logger custom library
add_subdirectory (logger)
include_directories("${CMAKE_SOURCE_DIR}/logger/src")

#Eigen libraries
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

# Compile main's
foreach(tool ${TOOLS})
	set(SOURCE_FILES tools/${tool}.cpp)
	add_executable(${tool} ${SOURCE_FILES})
	target_link_libraries(${tool} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} utils logger)
endforeach(tool)
