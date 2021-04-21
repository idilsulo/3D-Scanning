cmake_minimum_required(VERSION 3.10)

# Arbitrary name
project(cmake-tutorial)

# Set C++ standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)



# OPTIONS

# Add some options which appear in the GUI
set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to the libs folder")
set(MAGIC_TOGGLE True CACHE BOOL "Some toggable")

# This message will be display during the configuration of the project
if(${MAGIC_TOGGLE})
    message(STATUS "Magic Toggle is on.")
else()
    message(STATUS "Magic Toggle is off.")
endif()





# LIBRARIES

# Let's have a look how libraries are setup
LIST(APPEND LIB_FILES "lib/algo.h" "lib/algo.cpp")
add_library(lib STATIC ${LIB_FILES})



# EXECUTABLES

# Let's be explicit about the header and source files
LIST(APPEND HEADER_FILES "util.h" "constants.h")
LIST(APPEND SOURCE_FILES "main.cpp")

# Define an executable with a name and the files
add_executable(app ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(app PUBLIC "lib")  # Make the lib folder an include directory
target_link_libraries(app PUBLIC lib)  # Link this library to our executable

# Redirect standard output
execute_process(
    COMMAND ./app ${SOURCE_FILES} ${HEADER_FILES}
    OUTPUT_VARIABLE _testOut
)

# Write output to file
file (WRITE "output.txt" "${_testOut}")


