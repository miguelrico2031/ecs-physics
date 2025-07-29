#include "raylib.h"
#include "rlgl.h"
#include <Physics/World.h>
#include <random>
#include <cmath>
#include <iostream>

namespace custom
{
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
}

struct
{
	const int numBodies = 5;
	const epl::Vector3 startPos = { 0.f, 25.f, 0.f };
	const float spread = 20.f;
	const float bodiesRestitution = 0.75f;
} SimulationParams;

static bool paused = false;

void initWindowAndCamera(Camera& camera);
void registerCustomComponents(epl::Registry& reg);
void createFloor(epl::World& world);
void createBodies(epl::World& world);
void createBigCuboid(epl::World& world);

void update(epl::World& world, Camera& camera, float dt);
void render(epl::World& world, Camera& camera);

void renderColliders(const epl::Registry& reg);
void renderCollisionNormals(const epl::World& world);
void renderRaysAndHits(const epl::Registry& reg);
epl::Vector3 randomInUnitSphere();
epl::Vector3 randomEulerAngles();
void togglePause();
void toggleGravity(epl::Registry& reg);
void applyRandomUpForce(epl::Registry& reg);
void raycastAtMousePos(epl::World& world, Camera& camera, bool multiple);
void shootBody(epl::World& world, Camera& camera);
void addIsColliding(epl::World& world);

inline static Vector3 toRaylib(const epl::Vector3& v) { return Vector3{ v.x, v.y, v.z }; }
inline static epl::Vector3 toEpl(const Vector3& v) { return epl::Vector3{ v.x, v.y, v.z }; }

inline static Ray toRaylib(const epl::Ray& r) { return Ray{ toRaylib(r.origin), toRaylib(r.direction) }; }
inline static epl::Ray toEpl(const Ray& r) { return epl::Ray{ toEpl(r.position), toEpl(r.direction) }; }

int main()
{
	Camera camera;
	initWindowAndCamera(camera);

	std::shared_ptr<epl::Registry> regPtr = std::make_shared<epl::Registry>(2048);
	epl::World world(regPtr, .2f);

	registerCustomComponents(*regPtr);
	createFloor(world);
	createBodies(world);
	//createBigCuboid(world);

	const float fixedDelta = 1.f / 60.f;
	float accumulator = 0.f;
	double previousTime = GetTime();

	while (!WindowShouldClose())
	{
		double currentTime = GetTime();
		float deltaTime = (float)(currentTime - previousTime);
		previousTime = currentTime;
		accumulator += deltaTime;

		while (accumulator >= fixedDelta)
		{
			if (!paused)
			{
				//fixed update logic goes here
				//internal physics update
				world.step(fixedDelta, 4);
			}
			accumulator -= fixedDelta;
		}

		//input logic and update here
		update(world, camera, deltaTime);

		//render
		render(world, camera);
	}
	CloseWindow();
}


void initWindowAndCamera(Camera& camera)
{
	const int screenWidth = 1200;
	const int screenHeight = 800;
	InitWindow(screenWidth, screenHeight, "epl Raylib Demo");

	DisableCursor();

	camera.position = Vector3{ 0.f, 8.f, 20.f };
	camera.target = Vector3{ 0.f, 1.f, 0.f };
	camera.up = Vector3{ 0.f, 1.f, 0.f };
	camera.fovy = 45.f;
	camera.projection = CAMERA_PERSPECTIVE;
}

void registerCustomComponents(epl::Registry& reg)
{
	reg.registerComponentType<custom::IsColliding>();
	reg.registerComponentType<custom::CastedRay>();
	reg.registerComponentType<custom::RayHitPoint>();
}

void createFloor(epl::World& world)
{
	auto e = world.createKinematicBody(epl::Vector3{ 0.f, -2, 0.f });
	world.getRegistry().addComponent<epl::AABBCollider>(e, epl::Vector3{ 30.f, 2, 30.f });
}

void createBodies(epl::World& world)
{
	for (size_t i = 0; i < SimulationParams.numBodies; i++)
	{
		epl::Vector3 pos = SimulationParams.startPos + randomInUnitSphere() * SimulationParams.spread;
		auto e = world.createDynamicBody(1, pos);
		world.changeRestitution(e, SimulationParams.bodiesRestitution);
		
		//world.getRegistry().addComponent<epl::AABBCollider>(e, epl::Vector3{ .5f, .5f,.5f });

		if (i % 2 == 0)
		{
			world.addSphereColliderToBody(e, .5f);
		}
		else
		{
			world.addBoxColliderToBody(e, epl::Vector3{ .75f, .75f, .75f });
		}
		//auto angles = randomEulerAngles();
		//world.getRegistry().getComponent<epl::Torque>(e).value += epl::Vector3{ angles.x, angles.y, angles.z } *10.f;
	}
}

void createBigCuboid(epl::World& world)
{
	auto e = world.createDynamicBody(5.f, epl::Vector3::zero(), epl::Quaternion::identity(), epl::Vector3::zero());
	world.changeRestitution(e, SimulationParams.bodiesRestitution);
	//world.getRegistry().removeComponent<epl::Gravity>(e);
	world.addBoxColliderToBody(e, epl::Vector3{ 5, 1, 1 });
}

void update(epl::World& world, Camera& camera, float dt)
{
	if (IsKeyPressed(KEY_SPACE))
	{
		togglePause();
	}
	if (!paused)
	{
		if (IsKeyPressed(KEY_G))
		{
			toggleGravity(world.getRegistry());
		}
		if (IsKeyPressed(KEY_F))
		{
			applyRandomUpForce(world.getRegistry());
		}
		if (IsMouseButtonPressed(0))
		{
			raycastAtMousePos(world, camera, false);
		}
		else if (IsMouseButtonPressed(1))
		{
			raycastAtMousePos(world, camera, true);
		}
		else if (IsMouseButtonPressed(2))
		{
			shootBody(world, camera);
		}
	}

	addIsColliding(world);
}

void render(epl::World& world, Camera& camera)
{
	UpdateCamera(&camera, CAMERA_FREE);

	BeginDrawing();
	ClearBackground(BLACK);

	BeginMode3D(camera);
	renderCollisionNormals(world);
	renderColliders(world.getRegistry());
	renderRaysAndHits(world.getRegistry());
	DrawGrid(60, 1.0f);
	EndMode3D();
	DrawText("+", GetScreenWidth() / 2, GetScreenHeight() / 2, 20, WHITE);
	DrawFPS(10, 20);
	EndDrawing();
}



void renderColliders(const epl::Registry& reg)
{
	for (const auto& [entity, collider] : reg.iterate<epl::AABBCollider>())
	{
		Color color = reg.hasComponent<custom::IsColliding>(entity) ? LIME : ORANGE;
		epl::Vector3 position = reg.getComponent<epl::Position>(entity).value;
		DrawCubeWires({ position.x, position.y, position.z },
			collider.halfSize.x * 2.f, collider.halfSize.y * 2.f, collider.halfSize.z * 2.f, color);
	}
	for (const auto& [entity, collider] : reg.iterate<epl::SphereCollider>())
	{
		Color color = reg.hasComponent<custom::IsColliding>(entity) ? GREEN : RED;
		epl::Vector3 position = reg.getComponent<epl::Position>(entity).value;
		epl::Quaternion rotation = reg.getComponent<epl::Rotation>(entity).value;
		
		auto [axis, angleRadians] = epl::Quaternion::toAxisAngle(rotation);

		rlPushMatrix();
		rlTranslatef(position.x, position.y, position.z);
		rlRotatef(RAD2DEG * angleRadians, axis.x, axis.y, axis.z);
		DrawSphereWires({0, 0, 0}, collider.radius, 8, 10, color);
		rlPopMatrix();
	}
	
	for (const auto& [entity, collider] : reg.iterate<epl::BoxCollider>())
	{
		Color color = reg.hasComponent<custom::IsColliding>(entity) ? GREEN : RED;
		epl::Vector3 position = reg.getComponent<epl::Position>(entity).value;
		epl::Quaternion rotation = reg.getComponent<epl::Rotation>(entity).value;

		auto [axis, angleRadians] = epl::Quaternion::toAxisAngle(rotation);

		rlPushMatrix();
		rlTranslatef(position.x, position.y, position.z);
		rlRotatef(RAD2DEG * angleRadians, axis.x, axis.y, axis.z);
		DrawCubeWiresV({ 0, 0, 0 }, toRaylib(collider.halfSize * 2.f), color);
		rlPopMatrix();

		////also draw the box not rotated to debug
		//DrawCubeWires({ position.x, position.y, position.z },
		//	collider.halfSize.x * 2.f, collider.halfSize.y * 2.f, collider.halfSize.z * 2.f, LIGHTGRAY);
	}

}

void renderCollisionNormals(const epl::World& world)
{
	for (const auto& collision : world.getAllCollisions())
	{
		epl::Vector3 pos1 = world.getRegistry().getComponent<epl::Position>(collision.entity1).value;
		DrawLine3D(toRaylib(pos1), toRaylib(pos1 + collision.normal * 2.f), YELLOW);
		DrawSphere(toRaylib(collision.contactPoint), .05f, MAGENTA);
	}
}

void renderRaysAndHits(const epl::Registry& reg)
{
	for (const auto [entity, ray] : reg.iterate<custom::CastedRay>())
	{
		DrawRay(ray.ray, ray.hit ? BLUE : WHITE);
	}
	for (const auto [entity, hitPoint] : reg.iterate<custom::RayHitPoint>())
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

epl::Vector3 randomEulerAngles()
{
	static thread_local std::mt19937 gen(std::random_device{}());
	static thread_local std::uniform_real_distribution<float> dist(0.f, 2.f * epl::Math::pi());
	return { dist(gen), dist(gen), dist(gen) };
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
	epl::Ray ray = toEpl(raylibRay);


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
			reg.removeComponent<custom::RayHitPoint>(e);
		}
		hits.clear();
		world.raycastMultiple(ray, hits, maxHits);
		reg.addOrSetComponent<custom::CastedRay>(rayEntity, raylibRay, !hits.empty());
		for (size_t i = 0; i < hits.size(); i++)
		{
			epl::Entity e = rayHitPointEntities[i];
			reg.addComponent<custom::RayHitPoint>(e, toRaylib(hits[i].point));
		}
	}

	else
	{
		static epl::Entity rayHitPointEntity = reg.createEntity();
		epl::RayHit rayHit;
		bool hit = world.raycast(ray, rayHit);

		reg.addOrSetComponent<custom::CastedRay>(rayEntity, raylibRay, hit);

		if (hit)
		{
			reg.addOrSetComponent<custom::RayHitPoint>(rayHitPointEntity, toRaylib(rayHit.point));
		}
		else if (reg.hasComponent<custom::RayHitPoint>(rayHitPointEntity))
		{
			reg.removeComponent<custom::RayHitPoint>(rayHitPointEntity);
		}
	} 
}

void shootBody(epl::World& world, Camera& camera)
{
	constexpr float forceMagnitude = 400.f;
	epl::Registry& reg = world.getRegistry();
	Vector2 mousePos = { GetScreenWidth() / 2.f,  GetScreenHeight() / 2.f };
	Ray raylibRay = GetScreenToWorldRay(mousePos, camera);
	epl::Ray ray = toEpl(raylibRay);
	epl::RayHit rayHit;
	if (world.raycast(ray, rayHit) && reg.hasComponent<epl::Force>(rayHit.entity) && reg.hasComponent<epl::Torque>(rayHit.entity))
	{
		world.addForceAtPoint(rayHit.entity, ray.direction * forceMagnitude, rayHit.point);
	}
}

void addIsColliding(epl::World& world)
{
	for (auto& [e, isCol] : world.getRegistry().iterate<custom::IsColliding>())
	{
		world.getRegistry().removeComponent<custom::IsColliding>(e);
	}
	for (const auto& collision : world.getAllCollisions())
	{
		world.getRegistry().addOrSetComponent<custom::IsColliding>(collision.entity1);
		world.getRegistry().addOrSetComponent<custom::IsColliding>(collision.entity2);
	}
}

