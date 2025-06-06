#include <iostream>
#include <Physics/World.h>



int main()
{
	epl::World world(4096);

	auto& reg = world.getRegistry();

	float mass1 = 1, mass2 = 1;
	epl::Vector3 pos1 = { 0, 10, 0 }, pos2 = { 5, 10, -10 };

	epl::Entity e1 = world.createDynamicBody(mass1, pos1);
	epl::Entity e2 = world.createDynamicBody(mass2, pos2);

	reg.getComponent<epl::LinearVelocity>(e1).value = { 0, 2, 0 };

	std::cout << "Pos1: " << pos1.x << ", " << pos1.y << ", " << pos1.z << std::endl;
	std::cout << "Pos2: " << pos2.x << ", " << pos2.y << ", " << pos2.z << std::endl;
	std::cout << "SIMULATION RUN" << std::endl;

	for (size_t i = 0; i < 60; i++)
	{
		world.step(1.f / 60.f);
		std::cout << "Step: " << i + 1 << std::endl;
		pos1 = reg.getComponent<epl::Position>(e1).value;
		pos2 = reg.getComponent<epl::Position>(e2).value;
		std::cout << "Pos1: " << pos1.x << ", " << pos1.y << ", " << pos1.z << std::endl;
		std::cout << "Pos2: " << pos2.x << ", " << pos2.y << ", " << pos2.z << std::endl;
	}

}



//Intellisense is dumb, I have to help it figure out howe the component pool works
void unusedFunction()
{
	epl::ComponentPool<int> pool(10);
	pool.add(2, 90);
	pool.get(2);
	pool.getOptional(2);
	pool.remove(2);
}