#include <Physics/Motion/RotateInertiaTensorSystem.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Colliders/OBBCollider.h>

void epl::RotateInertiaTensorSystem::rotateInertiaTensors(Registry& reg)
{
	for (const auto& [entity, localInvInertia] : reg.iterate<LocalInverseInertia>())
	{
		const Quaternion& rotation = reg.getComponent<Rotation>(entity).value;
		auto rotatedInvInertia = OBBColliderFuncs::calculateRotatedInverseInertiaTensor(localInvInertia.tensor, rotation);
		reg.getComponent<InverseInertia>(entity).tensor = rotatedInvInertia;
	}
}
