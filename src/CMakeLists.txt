cmake_minimum_required (VERSION 3.0)
project (Raytracer)
set (CMAKE_CXX_STANDARD 11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# The version number
set (Raytracer_VERSION_MAJOR 0)
set (Raytracer_VERSION_MINOR 1)

# include Eigen3
#set (EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/eigen")
#include_directories ( "${EIGEN3_INCLUDE_DIR}" )
find_package (Eigen3 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# include OpenCV3
find_package( OpenCV REQUIRED)
include_directories( "${OpenCV_INCLUDE_DIRS}" )
include_directories( "${PROJECT_SOURCE_DIR}" )

add_executable(Raytracer    main.cpp
                            box3.cpp
                            camera.cpp
                            global.cpp
                            lighting.cpp
                            material.cpp
                            raytracer.cpp
                            sceneobject.cpp
                            trimesh.cpp )

target_link_libraries( Raytracer ${OpenCV_LIBS} Threads::Threads )


