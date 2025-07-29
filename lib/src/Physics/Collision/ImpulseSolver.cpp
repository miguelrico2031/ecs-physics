#include <Physics/Collision/ImpulseSolver.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	void ImpulseSolver::resolveCollisions(Registry& reg, const std::vector<Collision>& collisions)
	{
		for (const Collision& collision : collisions)
		{
			//TODO: maybe change the architecture so DynamicBody is not used and the dynamic check is = mass > 0
			//or alteratively remove mass component from kinematic bodies

 			bool isDynamic1 = reg.hasComponent<DynamicBody>(collision.entity1);
			bool isDynamic2 = reg.hasComponent<DynamicBody>(collision.entity2);

			if (!isDynamic1 && !isDynamic2) // kinematic - kinematic collisions won't be resolved
			{
				continue;
			}


			const Vector3& pos1 = reg.getComponent<Position>(collision.entity1).value;
			const Vector3& pos2 = reg.getComponent<Position>(collision.entity2).value;
			//positions relative to the contact point
			Vector3 relativePos1 = collision.contactPoint - pos1;
			Vector3 relativePos2 = collision.contactPoint - pos2;

			float invMass1 = 0.f;
			float invMass2 = 0.f;

			Vector3 totalVelocity1 = Vector3::zero();
			Vector3 totalVelocity2 = Vector3::zero();

			if (isDynamic1)
			{
				const Vector3& linVel1 = reg.getComponent<LinearVelocity>(collision.entity1).value;
				const Vector3& angVel1 = reg.getComponent<AngularVelocity>(collision.entity1).value;

				invMass1 = reg.getComponent<Mass>(collision.entity1).inverseMass;

				//linear velocity at the contact point caused by the angular velocity
				Vector3 rotationalVelocity1 = Vector3::cross(angVel1, relativePos1);
				totalVelocity1 = linVel1 + rotationalVelocity1;
			}

			if (isDynamic2)
			{
				const Vector3& linVel2 = reg.getComponent<LinearVelocity>(collision.entity2).value;
				const Vector3& angVel2 = reg.getComponent<AngularVelocity>(collision.entity2).value;

				invMass2 = reg.getComponent<Mass>(collision.entity2).inverseMass;

				//linear velocity at the contact point caused by the angular velocity
				Vector3 rotationalVelocity2 = Vector3::cross(angVel2, relativePos2);
				totalVelocity2 = linVel2 + rotationalVelocity2;
			}

			//relative velocity at contact point
			Vector3 relativeVelocity = totalVelocity2 - totalVelocity1;

			float velocityAlongNormal = Vector3::dot(relativeVelocity, collision.normal);


			Vector3 rotationalResistanceFromTorque1 = Vector3::zero();
			Vector3 rotationalResistanceFromTorque2 = Vector3::zero();

			if (isDynamic1)
			{
				const Matrix3x3& invInertia1 = reg.getComponent<InverseInertia>(collision.entity1).tensor;

				//direction of the torque that the impulse will cause at the relative position
				Vector3 torqueDirection1 = Vector3::cross(relativePos1, collision.normal);
				//how the body will rotate with the torque
				Vector3 angularResponse1 = invInertia1 * torqueDirection1;
				//movement resistance caused by the torque at the contac point
				rotationalResistanceFromTorque1 = Vector3::cross(angularResponse1, relativePos1);
			}

			if (isDynamic2)
			{
				const Matrix3x3& invInertia2 = reg.getComponent<InverseInertia>(collision.entity2).tensor;

				//direction of the torque that the impulse will cause at the relative position
				Vector3 torqueDirection2 = Vector3::cross(relativePos2, collision.normal);
				//how the body will rotate with the torque
				Vector3 angularResponse2 = invInertia2 * torqueDirection2;
				//movement resistance caused by the torque at the contac point
				rotationalResistanceFromTorque2 = Vector3::cross(angularResponse2, relativePos2);
			}

			//the movement resistance that the 2 bodies' torques will cause
			float totalRotationalResistance = Vector3::dot(rotationalResistanceFromTorque1 + rotationalResistanceFromTorque2, collision.normal);



			float totalInvMass = invMass1 + invMass2;

			float restitution1 = 0.f;
			if (const auto& matOpt1 = reg.tryGetComponent<PhysicMaterial>(collision.entity1))
			{
				restitution1 = matOpt1->restitution;
			}

			float restitution2 = 0.f;
			if (const auto& matOpt2 = reg.tryGetComponent<PhysicMaterial>(collision.entity2))
			{
				restitution2 = matOpt2->restitution;
			}

			float avgRestitution = (restitution1 + restitution2) * .5f;


			float impulseMagnitude = (1.f + avgRestitution) * -velocityAlongNormal;
			impulseMagnitude /= totalInvMass + totalRotationalResistance;

			Vector3 impulse = collision.normal * impulseMagnitude;

			if (isDynamic1)
			{
				LinearVelocity& linVel1 = reg.getComponent<LinearVelocity>(collision.entity1);
				AngularVelocity& angVel1 = reg.getComponent<AngularVelocity>(collision.entity1);
				const Matrix3x3& invInertia1 = reg.getComponent<InverseInertia>(collision.entity1).tensor;

				linVel1.value += -impulse * invMass1;
				Vector3 angularImpulse1 = Vector3::cross(relativePos1, impulse);
				angVel1.value += invInertia1 * -angularImpulse1;
			}

			if (isDynamic2)
			{
				LinearVelocity& linVel2 = reg.getComponent<LinearVelocity>(collision.entity2);
				AngularVelocity& angVel2 = reg.getComponent<AngularVelocity>(collision.entity2);
				const Matrix3x3& invInertia2 = reg.getComponent<InverseInertia>(collision.entity2).tensor;

				linVel2.value += impulse * invMass2;
				Vector3 angularImpulse2 = Vector3::cross(relativePos2, impulse);
				angVel2.value += invInertia2 * angularImpulse2;
			}
		}
	}
}
