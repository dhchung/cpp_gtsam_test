cmake_minimum_required(VERSION 3.0)
project(ship_hull_inspection)

set(SOURCE
    "src/main.cpp")

set(TEST_GTSAM
    "test/test.cpp")


file(GLOB CLASS_SOURCES
     "classes/*.cpp"
     "include/*.h")

include_directories(${PROJECT_SOURCE_DIR}/include)

#find gtsam library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#OpenGL
add_subdirectory(glad/)
set(LIB_SOURCES glad/src/glad.c)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.2 REQUIRED)

add_executable(${PROJECT_NAME} 
               ${SOURCE}
               ${LIB_SOURCES}
               ${CLASS_SOURCES})

add_executable(test_stuff 
               ${TEST_GTSAM}
               ${LIB_SOURCES}
               ${CLASS_SOURCES})

include_directories(${OPENGL_INCLUDE_DIRS} 
                    ${GLUT_INCLUDE_DIRS} 
                    ${GLM_INCLUDE_DIRS})


target_link_libraries (${PROJECT_NAME} 
                       gtsam
                       Eigen3::Eigen
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${OPENGL_LIBRARIES} 
                       ${GLUT_LIBRARIES} 
                       glfw 
                       ${GLEW_LIBRARIES} 
                       ${GLM_LIBRARY_DIRS}
                       glad ${CMAKE_DL_LIBS})


target_link_libraries (test_stuff
                       gtsam
                       Eigen3::Eigen
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${OPENGL_LIBRARIES} 
                       ${GLUT_LIBRARIES} 
                       glfw 
                       ${GLEW_LIBRARIES} 
                       ${GLM_LIBRARY_DIRS}
                       glad ${CMAKE_DL_LIBS})
