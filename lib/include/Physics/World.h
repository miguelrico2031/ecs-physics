#pragma once
#include <ECS/Entity.h>
#include <memory>
namespace epl
{
	class World
	{
		
	public:
		World(size_t maxEntities);
		World(std::shared_ptr<Registry> registry);

		Registry& getRegistry() { return *m_registry; }
		const Registry& getRegistry() const { return *m_registry; }
	private:
		std::shared_ptr<Registry> m_registry;
	};

}