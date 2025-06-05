#include <iostream>
#include <ECS/Registry.h>
#include <ECS/ComponentPool.h>
#include <Math/Vector3.h>
#include <Physics/Components.h>
#include <Physics/World.h>
int main()
{

	struct ExampleComponent
	{
		ExampleComponent(int v1, int v2) : value1(v1), value2(v2) {}
		~ExampleComponent()
		{
			std::cout << "destroying example component" << std::endl;	
		}
		void print() const
		{
			std::cout << "Value1: " << value1 << ", Value2: " << value2 << std::endl;
		}


		int value1;
		int value2;

	};

	using FloatComponent = float;

	constexpr size_t MAX_ENTITIES = 4096;
	epl::Registry<MAX_ENTITIES> registry;
	registry.registerComponentType<ExampleComponent>();
	registry.registerComponentType<FloatComponent>();
	epl::Entity entity = registry.createEntity();
	registry.addComponent<ExampleComponent>(entity, 10, 20);
	registry.addComponent<FloatComponent>(entity, 1.5f);

	const auto& creg = registry;
	const auto& ec = creg.getComponent<ExampleComponent>(entity);
	std::cout << "ExampleComponent values: ";
	ec.print();
	auto& fc = registry.getComponent<FloatComponent>(entity);
	std::cout << "FloatComponent value: " << fc << std::endl;


	epl::Entity entity2 = registry.createEntity();
	registry.addComponent<ExampleComponent>(entity2, 30, 40);
	std::cout << "new entity added.\n";

	for (auto [e, c] : registry.iterate<ExampleComponent>())
	{
		std::cout << "Entity " << e << " has ExampleComponent with values: ";
		c.print();
	}

	registry.registerComponentType<epl::Mass>();
	registry.addComponent<epl::Mass>(entity2, 1);

}

//Intellisense is dumb, I have to help it figure out howe the component pool works
void unusedFunction()
{
	epl::ComponentPool<int, 10> pool;
	pool.add(2, 90);
	pool.get(2);
	pool.getOptional(2);
	pool.remove(2);
}