#include <Physics/Motion/GravitySystem.h>

namespace epl
{
	void GravitySystem::applyGravity(Registry& registry)
	{
		for (const auto [entity, gravity] : registry.iterate<Gravity>())
		{
			Force& force = registry.getComponent <Force>(entity);
			const Mass& mass = registry.getComponent<Mass>(entity);
			force.value += gravity.value * mass.mass;
		}
	}
}