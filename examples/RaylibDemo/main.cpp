#include "raylib.h"
#include <Physics/World.h>
#include <random>
#include <cmath>
#include <iostream>

struct IsColliding {};
struct CastedRay
{
	Ray ray;
	bool hit;
	CastedRay(Ray ray_, bool hit_) : ray(ray_), hit(hit_) {}
};
struct RayHitPoint
{
	Vector3 point;
	RayHitPoint(Vector3 p) : point(p) {}
};

static bool paused = false;

void renderColliders(const epl::Registry& reg);
void renderCollisionNormals(const epl::World& world);
void renderRaysAndHits(const epl::Registry& reg);
epl::Vector3 randomInUnitSphere();
void togglePause();
void toggleGravity(epl::Registry& reg);
void applyRandomUpForce(epl::Registry& reg);
void raycastAtMousePos(epl::World& world, Camera& camera, bool multiple);
void addIsColliding(epl::World& world);

inline static Vector3 toVector3(const epl::Vector3& v) { return Vector3{ v.x, v.y, v.z }; }
inline static epl::Vector3 toVector3(const Vector3& v) { return epl::Vector3{ v.x, v.y, v.z }; }

inline static Ray toRay(const epl::Ray& r) { return Ray{ toVector3(r.origin), toVector3(r.direction) }; }
inline static epl::Ray toRay(const Ray& r) { return epl::Ray{ toVector3(r.position), toVector3(r.direction) }; }

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



	reg.registerComponentType<IsColliding>();
	reg.registerComponentType<CastedRay>();
	reg.registerComponentType<RayHitPoint>();



	epl::World world(regPtr);

	for (size_t i = 0; i < numBodies; i++)
	{
		epl::Vector3 pos = startPos + randomInUnitSphere() * spread;
		auto e = world.createDynamicBody(1, pos);

		if (i % 2 == 0)
		{
			reg.addComponent<epl::AABBCollider>(e, epl::Vector3{ .5f, .5f, .5f });
		}
		else
		{
			reg.addComponent<epl::SphereCollider>(e, .5f);
		}
	}

	const float fixedDelta = 1.f / 60.f;
	float accumulator = 0.f;
	double previousTime = GetTime();

	//SetTargetFPS(0); 

	while (!WindowShouldClose())
	{
		double currentTime = GetTime();
		float deltaTime = (float)(currentTime - previousTime);
		previousTime = currentTime;
		accumulator += deltaTime;



		if (IsKeyPressed(KEY_SPACE))
		{
			togglePause();
		}
		if (!paused)
		{
			if (IsKeyPressed(KEY_G))
			{
				toggleGravity(reg);
			}
			if (IsKeyPressed(KEY_F))
			{
				applyRandomUpForce(reg);
			}
			if (IsMouseButtonPressed(0))
			{
				raycastAtMousePos(world, camera, false);
			}
			else if (IsMouseButtonPressed(1))
			{
				raycastAtMousePos(world, camera, true);
			}
		}

		while (accumulator >= fixedDelta)
		{
			if (!paused)
			{
				world.step(fixedDelta, 4);
			}
			accumulator -= fixedDelta;
		}
		UpdateCamera(&camera, CAMERA_FREE);

		addIsColliding(world);

		BeginDrawing();
		ClearBackground(BLACK);

		BeginMode3D(camera);
		renderCollisionNormals(world);
		renderColliders(reg);
		renderRaysAndHits(reg);
		DrawGrid(50, 1.0f);
		EndMode3D();
		DrawText("+", GetScreenWidth() / 2, GetScreenHeight() / 2, 20, WHITE);
		DrawFPS(10, 20);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}


void renderColliders(const epl::Registry& reg)
{
	for (const auto& [entity, collider] : reg.iterate<epl::SphereCollider>())
	{
		Color color = reg.hasComponent<IsColliding>(entity) ? GREEN : RED;
		epl::Vector3 position = reg.getComponent<epl::Position>(entity).value + collider.offset;
		DrawSphereWires({ position.x, position.y, position.z }, collider.radius, 8, 10, color);
	}
	for (const auto& [entity, collider] : reg.iterate<epl::AABBCollider>())
	{
		Color color = reg.hasComponent<IsColliding>(entity) ? GREEN : RED;
		epl::Vector3 position = reg.getComponent<epl::Position>(entity).value + collider.offset;
		DrawCubeWires({ position.x, position.y, position.z },
			collider.halfSize.x * 2.f, collider.halfSize.y * 2.f, collider.halfSize.z * 2.f, color);
	}
}

void renderCollisionNormals(const epl::World& world)
{
	for (const auto& collision : world.m_collisions)
	{
		epl::Vector3 pos1 = world.getRegistry().getComponent<epl::Position>(collision.entity1).value;
		//epl::Vector3 pos2 = world.getRegistry().getComponent<epl::Position>(collision.entity2).value;
		DrawLine3D(toVector3(pos1), toVector3(pos1 + collision.normal * 2.f), YELLOW);
	}
}

void renderRaysAndHits(const epl::Registry& reg)
{
	for (const auto [entity, ray] : reg.iterate<CastedRay>())
	{
		DrawRay(ray.ray, ray.hit ? BLUE : WHITE);
	}
	for (const auto [entity, hitPoint] : reg.iterate<RayHitPoint>())
	{
		DrawSphere(hitPoint.point, .2f, VIOLET);
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

void togglePause()
{
	paused = !paused;
	std::cout << "Simulation " << (paused ? "paused" : "resumed") << std::endl;
}

void toggleGravity(epl::Registry& reg)
{
	static bool gravityEnabled = true;
	gravityEnabled = !gravityEnabled;
	for (auto [entity, gravity] : reg.iterate<epl::Gravity>())
	{
		if (gravityEnabled)
		{
			gravity.value = epl::Gravity::earth();
		}
		else
		{
			gravity.value = epl::Vector3::zero();
			reg.getComponent<epl::LinearVelocity>(entity).value = epl::Vector3::zero();
		}
	}
}


void applyRandomUpForce(epl::Registry& reg)
{
	constexpr float forceUpFactor = 20.f;
	constexpr float forceMagnitude = 4000.f;
	for (auto [entity, forceSum] : reg.iterate<epl::Force>())
	{
		epl::Vector3 forceDirection = randomInUnitSphere();
		forceDirection.y = epl::Math::abs(forceDirection.y) * forceUpFactor;
		forceSum.value += epl::Vector3::normalize(forceDirection) * forceMagnitude;
	}
}

void raycastAtMousePos(epl::World& world, Camera& camera, bool multiple)
{
	epl::Registry& reg = world.getRegistry();
	static epl::Entity rayEntity = reg.createEntity();
	Vector2 mousePos = { GetScreenWidth() / 2.f,  GetScreenHeight() / 2.f };
	Ray raylibRay = GetScreenToWorldRay(mousePos, camera);
	epl::Ray ray = toRay(raylibRay);


	if (multiple)
	{
		const size_t maxHits = 30;
		static bool init = false;
		static std::vector<epl::RayHit> hits;
		static std::vector<epl::Entity> rayHitPointEntities;
		if (!init)
		{
			init = true;
			hits.reserve(maxHits);
			rayHitPointEntities.reserve(maxHits);
			for (size_t i = 0; i < maxHits; i++)
			{
				rayHitPointEntities.push_back(reg.createEntity());
			}
		}
		for (size_t i = 0; i < hits.size(); i++)
		{
			epl::Entity e = rayHitPointEntities[i];
			reg.removeComponent<RayHitPoint>(e);
		}
		hits.clear();
		world.raycastMultiple(ray, hits, maxHits);
		reg.addOrSetComponent<CastedRay>(rayEntity, raylibRay, !hits.empty());
		for (size_t i = 0; i < hits.size(); i++)
		{
			epl::Entity e = rayHitPointEntities[i];
			reg.addComponent<RayHitPoint>(e, toVector3(hits[i].point));
		}
	}

	else
	{
		static epl::Entity rayHitPointEntity = reg.createEntity();
		epl::RayHit rayHit;
		bool hit = world.raycast(ray, rayHit);

		reg.addOrSetComponent<CastedRay>(rayEntity, raylibRay, hit);

		if (hit)
		{
			reg.addOrSetComponent<RayHitPoint>(rayHitPointEntity, toVector3(rayHit.point));
		}
		else if (reg.hasComponent<RayHitPoint>(rayHitPointEntity))
		{
			reg.removeComponent<RayHitPoint>(rayHitPointEntity);
		}
	}
}

void addIsColliding(epl::World& world)
{
	for (auto& [e, isCol] : world.getRegistry().iterate<IsColliding>())
	{
		world.getRegistry().removeComponent<IsColliding>(e);
	}
	for (const auto& collision : world.m_collisions)
	{
		world.getRegistry().addOrSetComponent<IsColliding>(collision.entity1);
		world.getRegistry().addOrSetComponent<IsColliding>(collision.entity2);
	}
}

