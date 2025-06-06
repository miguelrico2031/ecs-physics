
add_executable(RaylibDemo
    main.cpp
)

target_link_libraries(RaylibDemo
    PRIVATE
        ecsphysics
        raylib
)
