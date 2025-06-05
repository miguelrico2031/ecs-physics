cmake_minimum_required(VERSION 3.16)
project(ecsphysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# binaries (.exe) install location
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# fetch external deps

# Add this line before any add_subdirectory or target definitions
#add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)

# Subdirectories
add_subdirectory(lib)
add_subdirectory(examples/HelloWorld)
