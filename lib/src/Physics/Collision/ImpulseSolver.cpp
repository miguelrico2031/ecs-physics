#include <Physics/Collision/ImpulseSolver.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	void ImpulseSolver::resolveCollisions(Registry& reg, const std::vector<Collision>& collisions)
	{
		for (const Collision& collision : collisions)
		{
			bool isDynamic1 = reg.hasComponent<DynamicBody>(collision.entity1);
			bool isDynamic2 = reg.hasComponent<DynamicBody>(collision.entity2);

			if (!isDynamic1 && !isDynamic2) // kinematic - kinematic collisions won't be resolved
			{
				continue;
			}



#pragma region ATTRIBUTES_CACHING
			//cache the components and attributes that dont change with each contact point

			const Vector3& pos1 = reg.getComponent<Position>(collision.entity1).value;
			const Vector3& pos2 = reg.getComponent<Position>(collision.entity2).value;

			Vector3 initialLinVel1 = Vector3::zero();
			Vector3 initialLinVel2 = Vector3::zero();

			Vector3 initialAngVel1 = Vector3::zero();
			Vector3 initialAngVel2 = Vector3::zero();

			float invMass1 = 0.f;
			float invMass2 = 0.f;

			if (isDynamic1)
			{
				initialLinVel1 = reg.getComponent<LinearVelocity>(collision.entity1).value;
				initialAngVel1 = reg.getComponent<AngularVelocity>(collision.entity1).value;
				invMass1 = reg.getComponent<Mass>(collision.entity1).inverseMass;
			}

			if (isDynamic2)
			{
				initialLinVel2 = reg.getComponent<LinearVelocity>(collision.entity2).value;
				initialAngVel2 = reg.getComponent<AngularVelocity>(collision.entity2).value;
				invMass2 = reg.getComponent<Mass>(collision.entity2).inverseMass;
			}

			float totalInvMass = invMass1 + invMass2;

			float restitution1 = 0.f;
			float restitution2 = 0.f;
			float friction1 = .5f;
			float friction2 = .5f;

			if (const auto& matOpt1 = reg.tryGetComponent<PhysicMaterial>(collision.entity1))
			{
				restitution1 = matOpt1->restitution;
				friction1 = matOpt1->friction;
			}

			if (const auto& matOpt2 = reg.tryGetComponent<PhysicMaterial>(collision.entity2))
			{
				restitution2 = matOpt2->restitution;
				friction2 = matOpt2->friction;
			}

			float combinedRestitution = (restitution1 + restitution2) * .5f;
			float combinedFriction = Math::sqrt(friction1 * friction2);

			const auto& invInertiaOpt1 = reg.tryGetComponent<InverseInertia>(collision.entity1);
			const auto& invInertiaOpt2 = reg.tryGetComponent<InverseInertia>(collision.entity2);


			//velocity accumulators so the actual velocities aren't changed in the loop
			Vector3 linVel1Acc = Vector3::zero();
			Vector3 angVel1Acc = Vector3::zero();
			Vector3 linVel2Acc = Vector3::zero();
			Vector3 angVel2Acc = Vector3::zero();

#pragma endregion

			//apply an impulse for each contact point
			for (size_t i = 0; i < collision.contactPointsCount; i++)
			{
				Vector3 contactPoint = collision.contactPoints[i];

				Vector3 rotationalResistanceFromTorque1 = Vector3::zero();
				Vector3 rotationalResistanceFromTorque2 = Vector3::zero();

#pragma region RELATIVE_VELOCITY
				//calculate the relative velocity at the contact point

				//positions relative to the contact point
				Vector3 relativePos1 = contactPoint - pos1;
				Vector3 relativePos2 = contactPoint - pos2;

				Vector3 totalVelocity1 = Vector3::zero();
				Vector3 totalVelocity2 = Vector3::zero();

				if (isDynamic1)
				{
					//linear velocity at the contact point caused by the angular velocity
					Vector3 rotationalVelocity1 = Vector3::cross(initialAngVel1, relativePos1);
					totalVelocity1 = initialLinVel1 + rotationalVelocity1;
				}

				if (isDynamic2)
				{
					//linear velocity at the contact point caused by the angular velocity
					Vector3 rotationalVelocity2 = Vector3::cross(initialAngVel2, relativePos2);
					totalVelocity2 = initialLinVel2 + rotationalVelocity2;
				}

				Vector3 relativeVelocity = totalVelocity2 - totalVelocity1;

				float velocityAlongNormal = Vector3::dot(relativeVelocity, collision.normal);

#pragma endregion

#pragma region NORMAL_IMPULSE
				//calculate the normal impulse to separate the bodies 

				if (isDynamic1)
				{
					//direction of the torque that the impulse will cause at the relative position
					Vector3 torqueDirection1 = Vector3::cross(relativePos1, collision.normal);
					//how the body will rotate with the torque
					Vector3 angularResponse1 = invInertiaOpt1->tensor * torqueDirection1;
					//movement resistance caused by the torque at the contac point
					rotationalResistanceFromTorque1 = Vector3::cross(angularResponse1, relativePos1);
				}

				if (isDynamic2)
				{
					//direction of the torque that the impulse will cause at the relative position
					Vector3 torqueDirection2 = Vector3::cross(relativePos2, collision.normal);
					//how the body will rotate with the torque
					Vector3 angularResponse2 = invInertiaOpt2->tensor * torqueDirection2;
					//movement resistance caused by the torque at the contac point
					rotationalResistanceFromTorque2 = Vector3::cross(angularResponse2, relativePos2);
				}

				//the movement resistance that the 2 bodies' torques will cause
				float totalRotationalResistance =
					Vector3::dot(rotationalResistanceFromTorque1 + rotationalResistanceFromTorque2, collision.normal);



				float normalImpulseMagnitude = (1.f + combinedRestitution) * -velocityAlongNormal;
				normalImpulseMagnitude /= totalInvMass + totalRotationalResistance;

				Vector3 normalImpulse = collision.normal * normalImpulseMagnitude;

				if (isDynamic1)
				{
					linVel1Acc += -normalImpulse * invMass1;
					Vector3 angularImpulse1 = Vector3::cross(relativePos1, normalImpulse);
					angVel1Acc += invInertiaOpt1->tensor * -angularImpulse1;
				}

				if (isDynamic2)
				{
					linVel2Acc += normalImpulse * invMass2;
					Vector3 angularImpulse2 = Vector3::cross(relativePos2, normalImpulse);
					angVel2Acc += invInertiaOpt2->tensor * angularImpulse2;
				}
#pragma endregion

#pragma region FRICTION_TANGENT_IMPULSE
				//calculate the impulse caused by friction
				//some variables used in the normal impulse calc are reused because they mean the same but in the friction impulse context

				if (velocityAlongNormal > 0.f)
				{
					continue;
				}

				Vector3 tangent = relativeVelocity - collision.normal * velocityAlongNormal;

				if (Vector3::squaredMagnitude(tangent) < Math::epsilonSquared())
				{
					continue;
				}

				tangent = Vector3::normalize(tangent);

				float velocityAlongTangent = Vector3::dot(relativeVelocity, tangent);

				rotationalResistanceFromTorque1 = Vector3::zero();
				rotationalResistanceFromTorque2 = Vector3::zero();

				if (isDynamic1)
				{
					Vector3 torqueDirection1 = Vector3::cross(relativePos1, tangent);
					Vector3 angularResponse1 = invInertiaOpt1->tensor * torqueDirection1;
					rotationalResistanceFromTorque1 = Vector3::cross(angularResponse1, relativePos1);
				}

				if (isDynamic2)
				{
					Vector3 torqueDirection2 = Vector3::cross(relativePos2, tangent);
					Vector3 angularResponse2 = invInertiaOpt2->tensor * torqueDirection2;
					rotationalResistanceFromTorque2 = Vector3::cross(angularResponse2, relativePos2);
				}

				totalRotationalResistance = 
					Vector3::dot(rotationalResistanceFromTorque1 + rotationalResistanceFromTorque2, tangent);

				float tangentImpulseMagnitude = -velocityAlongTangent;
				tangentImpulseMagnitude /= totalInvMass + totalRotationalResistance;

				//clamp using coulomb formula : |tangent impulse magnitude| <= combinedFriction * normal impulse

				float maxFriction = combinedFriction * Math::max(0.f, normalImpulseMagnitude);

				tangentImpulseMagnitude = Math::clamp(tangentImpulseMagnitude, -maxFriction, maxFriction);

				Vector3 tangentImpulse = tangent * tangentImpulseMagnitude;

				if (isDynamic1)
				{
					linVel1Acc += -tangentImpulse * invMass1;
					Vector3 angularImpulse1 = Vector3::cross(relativePos1, tangentImpulse);

					//this avoids jittering on low torques
					if (Vector3::squaredMagnitude(angularImpulse1) > .001f)
					{
						angVel1Acc += invInertiaOpt1->tensor * -angularImpulse1;
						angVel1Acc *= .95f;
						if (angVel1Acc.x * angVel1Acc.x < .001f) angVel1Acc.x = 0.f;
						if (angVel1Acc.y * angVel1Acc.y < .001f) angVel1Acc.y = 0.f;
						if (angVel1Acc.z * angVel1Acc.z < .001f) angVel1Acc.z = 0.f;
					}

				}
				
				if (isDynamic2)
				{
					linVel2Acc += tangentImpulse * invMass2;
					Vector3 angularImpulse2 = Vector3::cross(relativePos2, tangentImpulse);
					if (Vector3::squaredMagnitude(angularImpulse2) > .01f)
					{
						angVel2Acc += invInertiaOpt2->tensor * angularImpulse2;
						angVel2Acc *= .95f;
						if (angVel2Acc.x * angVel2Acc.x < .001f) angVel2Acc.x = 0.f;
						if (angVel2Acc.y * angVel2Acc.y < .001f) angVel2Acc.y = 0.f;
						if (angVel2Acc.z * angVel2Acc.z < .001f) angVel2Acc.z = 0.f;
					}
				}

#pragma endregion

			}

			//apply the velocity accumulators to the actual components
			if (isDynamic1)
			{
				reg.getComponent<LinearVelocity>(collision.entity1).value += linVel1Acc;
				reg.getComponent<AngularVelocity>(collision.entity1).value += angVel1Acc;
			}
			if (isDynamic2)
			{
				reg.getComponent<LinearVelocity>(collision.entity2).value += linVel2Acc;
				reg.getComponent<AngularVelocity>(collision.entity2).value += angVel2Acc;
			}
		}
	}
}
