#pragma once
#include <ECS/Entity.h>

namespace epl
{
	template <Entity MAX_ENTITIES>
	class World
	{
		
	public:
		Registry<MAX_ENTITIES>& getRegistry() { return m_registry; }
		const Registry<MAX_ENTITIES>& getRegistry() const { return m_registry; }
	private:
		Registry<MAX_ENTITIES> m_registry;
	};

	using DefaultWorld = World<4096>;
}