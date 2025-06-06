#include "raylib.h"
#include <Physics/World.h>
#include <random>
#include <cmath>

struct Cube
{
    Color color; Vector3 size;
    Cube(Color c, Vector3 s) : color(c), size(s) {}
};

void renderCubes(const epl::Registry& reg);
epl::Vector3 randomInUnitSphere();


int main()
{
    const int screenWidth = 1200;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "epl Raylib Demo");

    DisableCursor();

    Camera camera;
    camera.position = Vector3{ 0.f, 8.f, 20.f }; 
    camera.target = Vector3{ 0.f, 1.f, 0.f };   
    camera.up = Vector3{ 0.f, 1.f, 0.f };       
    camera.fovy = 45.f;                           
    camera.projection = CAMERA_PERSPECTIVE;        


    constexpr int numBodies = 100;
    constexpr epl::Vector3 startPos = { 0.f, 25.f, 0.f };
	constexpr float spread = 5.f;


	std::shared_ptr<epl::Registry> regPtr = std::make_shared<epl::Registry>(2048);
	epl::Registry& reg = *regPtr;
	reg.registerComponentType<Cube>();
	epl::World world(regPtr);

    for (size_t i = 0; i < numBodies; i++)
    {
        epl::Vector3 pos = startPos + randomInUnitSphere() * spread;
        auto e = world.createDynamicBody(1, pos);

        reg.addComponent<Cube>(e, RED, Vector3{ .5f, .5f, .5f });
    }

    const float fixedDelta = 1.f / 60.f; 
    float accumulator = 0.f;
    double previousTime = GetTime();

    //SetTargetFPS(0); 

    while (!WindowShouldClose())
    {
        double currentTime = GetTime();
        float frameTime = (float)(currentTime - previousTime);
        previousTime = currentTime;
        accumulator += frameTime;

        while (accumulator >= fixedDelta)
        {
            UpdateCamera(&camera, CAMERA_FREE);
            world.step(fixedDelta, 4);
            accumulator -= fixedDelta;
        }

        BeginDrawing();
            ClearBackground(BLACK);

            BeginMode3D(camera);
            renderCubes(reg);
                DrawGrid(50, 1.0f);
            EndMode3D();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}

void renderCubes(const epl::Registry& reg)
{
    for (const auto [entity, cube] : reg.iterate<Cube>())
    {
        epl::Vector3 position = reg.getComponent<epl::Position>(entity).value;
        DrawCube({ position.x, position.y, position.z }, cube.size.x, cube.size.y, cube.size.z, cube.color);
    }
}

epl::Vector3 randomInUnitSphere()
{
    static thread_local std::mt19937 gen(std::random_device{}());
    static thread_local std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    epl::Vector3 p{};
    do {
        p.x = dist(gen);
        p.y = dist(gen);
        p.z = dist(gen);
    } while ((p.x * p.x + p.y * p.y + p.z * p.z) >= 1.0f);
    return p;
}