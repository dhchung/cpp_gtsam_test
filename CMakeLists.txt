cmake_minimum_required(VERSION 3.0)
project(testing_gtsam)

set(SOURCES helloworld.cpp)

add_executable(${PROJECT_NAME} 
               ${SOURCES})

#find gtsam library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

target_link_libraries (${PROJECT_NAME} 
                       gtsam
                       Eigen3::Eigen)
