#include <Physics/Colliders/OBBCollider.h>
#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Motion/MotionComponents.h>
#include <ECS/Registry.h>
#include <Math/Matrix3x3.h>
#include <vector>
#include <iostream>

namespace epl
{
	bool OBBColliderFuncs::isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		const Vector3& position1 = reg.getComponent<Position>(e1).value;
		const Vector3& position2 = reg.getComponent<Position>(e2).value;
		const Quaternion& rotation1 = reg.getComponent<Rotation>(e1).value;
		const Quaternion& rotation2 = reg.getComponent<Rotation>(e2).value;

		Vector3 axes1[3]
		{
			Quaternion::rotate(rotation1, {1, 0, 0}),
			Quaternion::rotate(rotation1, {0, 1, 0}),
			Quaternion::rotate(rotation1, {0, 0, 1})
		};
		Vector3 axes2[3]
		{
			Quaternion::rotate(rotation2, {1, 0, 0}),
			Quaternion::rotate(rotation2, {0, 1, 0}),
			Quaternion::rotate(rotation2, {0, 0, 1})
		};


		Vector3 direction = position2 - position1;


		if (testAllSeparatingAxes(axes1, axes2, c1.halfSize, c2.halfSize, direction, col))
		{
			return false;
		}

		//No separating axis found, there is a collision

		col.entity1 = e1;
		col.entity2 = e2;
		if (Vector3::dot(direction, col.normal) < 0.0f)
		{
			col.normal = -col.normal;
		}



		//Find the contact points manifold, simpler approach than clipping, the points are found by
		//detecting all of the vertices of each box that are inside the other box

		Vector3 vertices1[8];
		Vector3 vertices2[8];
		getBoxVertices(position1, c1.halfSize, axes1, vertices1);
		getBoxVertices(position2, c2.halfSize, axes2, vertices2);

		Quaternion inverseRotation1 = Quaternion::conjugate(rotation1);
		Quaternion inverseRotation2 = Quaternion::conjugate(rotation2);

		std::vector<Vector3> contactPoints;
		contactPoints.reserve(8);

		//check which vertices of the box 1 are inside the box 2
		for (const Vector3& vertex1 : vertices1)
		{
			//transform the vertex to the obb local space so it acts as an aabb
			Vector3 localPoint = Quaternion::rotate(inverseRotation2, vertex1 - position2);
			if (AABBColliderFuncs::isPointInsideBox(localPoint, c2.halfSize))
			{
				contactPoints.push_back(vertex1);
			}
		}

		//check which vertices of the box 2 are inside the box 1
		for (const Vector3& vertex2 : vertices2)
		{
			//transform the vertex to the obb local space so it acts as an aabb
			Vector3 localPoint = Quaternion::rotate(inverseRotation1, vertex2 - position1);
			if (AABBColliderFuncs::isPointInsideBox(localPoint, c1.halfSize))
			{
				contactPoints.push_back(vertex2);
			}
		}

		//assert(!contactPoints.empty() && "No contact points found on collision");

		// Get the 4 contact points closer to the collision center
		if (contactPoints.size() > 4)
		{
			Vector3 center = (position1 + position2) * 0.5f;
			std::sort(contactPoints.begin(), contactPoints.end(), [&](const Vector3& a, const Vector3& b)
				{
					return Vector3::squaredMagnitude(a - center) < Vector3::squaredMagnitude(b - center);
				});
			contactPoints.resize(4);
		}
		col.contactPointsCount = contactPoints.size();
		for (size_t i = 0; i < contactPoints.size(); i++)
		{
			col.contactPoints[i] = contactPoints[i];
		}
		

		return true;
	}


	bool OBBColliderFuncs::isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 position1 = reg.getComponent<Position>(e1).value;
		Vector3 position2 = reg.getComponent<Position>(e2).value;
		Quaternion rotation1 = reg.getComponent<Rotation>(e1).value;


		Vector3 axes1[3]
		{
			Quaternion::rotate(rotation1, {1, 0, 0}),
			Quaternion::rotate(rotation1, {0, 1, 0}),
			Quaternion::rotate(rotation1, {0, 0, 1})
		};

		static constexpr Vector3 axes2[3] //AABB's axes are aligned with the world space axes
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}
		};


		Vector3 direction = position2 - position1;

		if (testAllSeparatingAxes(axes1, axes2, c1.halfSize, c2.halfSize, direction, col))
		{
			return false;
		}

		//No separating axis found, there is a collision

		col.entity1 = e1;
		col.entity2 = e2;
		if (Vector3::dot(direction, col.normal) < 0.0f)
		{
			col.normal = -col.normal;
		}



		//Find the contact points manifold

		Vector3 vertices1[8];
		Vector3 vertices2[8];
		getBoxVertices(position1, c1.halfSize, axes1, vertices1);
		getBoxVertices(position2, c2.halfSize, axes2, vertices2);

		Quaternion inverseRotation1 = Quaternion::conjugate(rotation1);

		std::vector<Vector3> contactPoints;
		contactPoints.reserve(8);

		//check which vertices of the box 1(obb) are inside the box 2(aabb)
		for (const Vector3& vertex1 : vertices1)
		{
			//no need to rotate, only transform to aabb origin space
			Vector3 localPoint = vertex1 - position2;
			if (AABBColliderFuncs::isPointInsideBox(localPoint, c2.halfSize))
			{
				contactPoints.push_back(vertex1);
			}
		}

		//check which vertices of the box 2(aabb) are inside the box 1(obb)
		for (const Vector3& vertex2 : vertices2)
		{
			//transform the vertex to the obb local space so it acts as an aabb
			Vector3 localPoint = Quaternion::rotate(inverseRotation1, vertex2 - position1);
			if (AABBColliderFuncs::isPointInsideBox(localPoint, c1.halfSize))
			{
				contactPoints.push_back(vertex2);
			}
		}

		// Get the 4 contact points closer to the collision center
		if (contactPoints.size() > 4)
		{
			Vector3 center = (position1 + position2) * 0.5f;
			std::sort(contactPoints.begin(), contactPoints.end(), [&](const Vector3& a, const Vector3& b)
				{
					return Vector3::squaredMagnitude(a - center) < Vector3::squaredMagnitude(b - center);
				});
			contactPoints.resize(4);
		}
		col.contactPointsCount = contactPoints.size();
		for (size_t i = 0; i < contactPoints.size(); i++)
		{
			col.contactPoints[i] = contactPoints[i];
		}



		return true;
	}

	bool OBBColliderFuncs::isCollidingSphereOBB(const Registry& reg, const SphereCollider& c1, const OBBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 spherePosition = reg.getComponent<Position>(e1).value;
		Vector3 obbPosition = reg.getComponent<Position>(e2).value;
		Quaternion obbRotation = reg.getComponent<Rotation>(e2).value;

		Matrix3x3 obbLocalToWorldSpaceTransform = Matrix3x3(obbRotation);
		Matrix3x3 worldToObbLocalSpaceTransform = Matrix3x3(Quaternion::conjugate(obbRotation));

		//sphere position in obb's local space
		Vector3 localSpherePos = worldToObbLocalSpaceTransform * (spherePosition - obbPosition);

		if (!SphereColliderFuncs::isCollidingSphereBox(localSpherePos, c1.radius, Vector3::zero(), c2.halfSize, col))
		{
			return false;
		}

		//transform results back to world space
		col.contactPoints[0] = (obbLocalToWorldSpaceTransform * col.contactPoints[0]) + obbPosition;


		col.normal = Vector3::normalize(obbLocalToWorldSpaceTransform * col.normal);
		col.entity1 = e1;
		col.entity2 = e2;
		return true;
	}



	bool OBBColliderFuncs::isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit)
	{
		//this method does a vector space basis change to align the axes with the box roation and place the box at local 0 0 0.
		//doing this allows us to treat the box as an AABB and use it's intersection check function.
		//to avoid using 4x4 transform matrices we use 3x3s to perform the rotations of the basis change, and for the translation transform
		//we translate the ray origin by subtracting the box origin, and then adding it again to the hit point
		//scale is not supported (yet) so this way we do the same as 4x4 transform matrices with 3x3s and a vector 

		Vector3 position = reg.getComponent<Position>(entity).value;
		Quaternion rotation = reg.getComponent<Rotation>(entity).value;
		Matrix3x3 localToWorldSpaceTransform = Matrix3x3(rotation);
		Matrix3x3 worldToLocalSpaceTransform = Matrix3x3(Quaternion::conjugate(rotation));

		Vector3 localRayOrigin = ray.origin - position;

		Ray rayInLocalSpace = { worldToLocalSpaceTransform * localRayOrigin, worldToLocalSpaceTransform * ray.direction };
		bool intersected = AABBColliderFuncs::isIntersectingBox(rayInLocalSpace, Vector3::zero(), collider.halfSize, hit);

		if (intersected)
		{
			hit.point = (localToWorldSpaceTransform * hit.point) + position;
		}
		return intersected;
	}


	bool OBBColliderFuncs::testAllSeparatingAxes(const Vector3 box1Axes[3], const Vector3 box2Axes[3], const Vector3& box1HalfSize,
		const Vector3& box2HalfSize, const Vector3& direction, Collision& col)
	{
		col.normal = Vector3::zero();
		col.depth = Math::infinity();

		//Box 1 axes
		if (testSeparatingAxis(box1Axes[0], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}
		if (testSeparatingAxis(box1Axes[1], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}
		if (testSeparatingAxis(box1Axes[2], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}

		//Box 2 axes
		if (testSeparatingAxis(box2Axes[0], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}
		if (testSeparatingAxis(box2Axes[1], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}
		if (testSeparatingAxis(box2Axes[2], box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
		{
			return true;
		}

		//Cross products of axes for edge axes
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0; j < 3; j++)
			{
				Vector3 axis = Vector3::cross(box1Axes[i], box2Axes[j]);
				if (testSeparatingAxis(axis, box1Axes, box1HalfSize, box2Axes, box2HalfSize, direction, col))
				{
					return true;
				}
			}
		}

		return false;
	}


	bool OBBColliderFuncs::testSeparatingAxis(Vector3 axis, const Vector3 box1Axes[3],
		const Vector3& box1HalfSize, const Vector3 box2Axes[3],
		const Vector3& box2HalfSize, const Vector3& direction, Collision& col)
	{

		float squaredMagnitude = Vector3::squaredMagnitude(axis);
		if (Math::equalsZero(squaredMagnitude, .000001f))
		{
			return false; // Degenerate axis, skip
		}

		axis /= Math::sqrt(squaredMagnitude); // Normalize the axis

		// Project both OBBs onto the axis
		float projection1 = projectBox(axis, box1Axes, box1HalfSize);
		float projection2 = projectBox(axis, box2Axes, box2HalfSize);

		// Project the distance between centers onto the axis
		float centerDistance = Vector3::dot(axis, direction);

		// total extent on the axis
		float totalProjection = projection1 + projection2;

		// If the distance is greater than the combined half-extents, there is a separating axis
		if (Math::abs(centerDistance) > totalProjection)
		{
			return true;
		}

		// Overlap is the amount of intersection along the axis
		float overlap = totalProjection - Math::abs(centerDistance);

		// Store the smallest overlap so far
		if (overlap < col.depth)
		{
			col.depth = overlap;
			col.normal = axis;
		}

		return false;
	}


	float OBBColliderFuncs::projectBox(const Vector3& axisToProject, const Vector3 boxAxes[3], const Vector3& boxHalfSize)
	{
		return	Math::abs(Vector3::dot(axisToProject, boxAxes[0]) * boxHalfSize.x) +
			Math::abs(Vector3::dot(axisToProject, boxAxes[1]) * boxHalfSize.y) +
			Math::abs(Vector3::dot(axisToProject, boxAxes[2]) * boxHalfSize.z);
	}


	void OBBColliderFuncs::getBoxVertices(const Vector3& position, const Vector3& halfSize, const Vector3 axes[3], Vector3 vertices[8])
	{
		int i = 0;
		for (int x = -1; x <= 1; x += 2)
		{
			for (int y = -1; y <= 1; y += 2)
			{
				for (int z = -1; z <= 1; z += 2)
				{
					vertices[i++] = position + axes[0] * (x * halfSize.x) + axes[1] * (y * halfSize.y) + axes[2] * (z * halfSize.z);
				}
			}
		}
	}



	Matrix3x3 OBBColliderFuncs::calculateRotatedInverseInertiaTensor(const Matrix3x3& localInvInertia, const Quaternion& rotation)
	{
		Matrix3x3 rotationMatrix(rotation);
		Matrix3x3 rotationMatrixTransposed = Matrix3x3::transpose(rotationMatrix);
		return rotationMatrix * localInvInertia * rotationMatrixTransposed;
	}

}