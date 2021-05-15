cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_03)

# Options
# set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
# set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")
set(Eigen3_INCLUDE_DIR "/usr/local/include/eigen3/")


# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

# Include freeimage directory
set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage/Source/)


# Define header and source files
set(HEADERS
    Eigen.h
    ProcrustesAligner.h
    SimpleMesh.h
)

set(SOURCES
    main.cpp
)

add_executable(exercise_03 ${HEADERS} ${SOURCES})
target_include_directories(exercise_03 PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})

# Link dylib
target_link_libraries(exercise_03 /usr/local/lib/libfreeimage.dylib)

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_03)
set_property(TARGET exercise_03 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
