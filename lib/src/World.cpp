#include <Physics/World.h>

namespace epl
{
	World::World(size_t maxEntities)
		: m_registry(std::make_shared<Registry>(maxEntities))
	{
	}

	World::World(std::shared_ptr<Registry> registry)
		: m_registry(registry)
	{
	}
}