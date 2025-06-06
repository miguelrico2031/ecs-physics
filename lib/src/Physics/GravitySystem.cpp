#include <Physics/GravitySystem.h>

namespace epl
{
	void GravitySystem::applyGravity(Registry& registry)
	{
		for (const auto [entity, gravity] : registry.iterate<Gravity>())
		{
			ForceSum& forceSum = registry.getComponent <ForceSum>(entity);
			const Mass& mass = registry.getComponent<Mass>(entity);
			forceSum.value += gravity.value * mass.mass;
		}
	}
}