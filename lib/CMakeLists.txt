
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE LIB_HEADERS CONFIGURE_DEPENDS include/**/*.h)

add_library(ecsphysics STATIC ${LIB_SOURCES} ${LIB_HEADERS} )
target_include_directories(ecsphysics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
