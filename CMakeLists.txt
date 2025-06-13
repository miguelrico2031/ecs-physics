cmake_minimum_required(VERSION 3.16)
project(ecsphysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# binaries (.exe) install location
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# fetch external deps
include(FetchContent)

# Adding Raylib
set(FETCHCONTENT_QUIET FALSE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE) # don't build the supplied example games
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG master
)

FetchContent_MakeAvailable(raylib)




# Subdirectories
add_subdirectory(lib)
add_subdirectory(examples/HelloWorld)
add_subdirectory(examples/RaylibDemo)
add_subdirectory(examples/FeatureTesting)
