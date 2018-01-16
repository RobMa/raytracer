cmake_minimum_required (VERSION 3.1)
project (Raytracer)
set (CMAKE_CXX_STANDARD 11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set( Raytracer_LIBS Threads::Threads )
find_package(Threads REQUIRED)

# The version number
set (Raytracer_VERSION_MAJOR 0)
set (Raytracer_VERSION_MINOR 1)

# include Eigen3
find_package (Eigen3 3.3 REQUIRED NO_MODULE )
set ( Raytracer_LIBS ${Raytracer_LIBS} Eigen3::Eigen )

# include OpenCV3
find_package( OpenCV REQUIRED)
include_directories( "${OpenCV_INCLUDE_DIRS}" )
set( Raytracer_LIBS ${Raytracer_LIBS} ${OpenCV_LIBS} )

# build executable
add_executable(Raytracer    main.cpp
                            box3.cpp
                            camera.cpp
                            global.cpp
                            lighting.cpp
                            material.cpp
                            raytracer.cpp
                            sceneobject.cpp
                            trimesh.cpp )

target_link_libraries( Raytracer ${Raytracer_LIBS} )
