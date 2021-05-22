cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)
project(exercise_04)
set(CMAKE_CXX_STANDARD 14)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# Define header and source files
set(UTILS
    utils/io.h
    utils/points.h
)

add_executable(gaussian ${UTILS} gaussian.cpp)
add_executable(dragon ${UTILS} dragon.cpp)
add_executable(surface ${UTILS} surface.cpp)

target_link_libraries(gaussian ceres)
target_link_libraries(dragon ceres)
target_link_libraries(surface ceres)

