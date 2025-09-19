#include <Physics/Colliders/ColliderFuncs.h>
#include <Physics/Collision/Collision.h>

#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Motion/MotionComponents.h>
#include <ECS/Registry.h>
#include <Math/Math.h>
#include <vector>

namespace epl
{
	constexpr float BOUNDS_PADDING = .0f;

#pragma region HELPERS

	bool ColliderFuncs::isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit)
	{
		Vector3 minPos = position - halfSize;
		Vector3 maxPos = position + halfSize;

		Vector3 rayIntersectionDistances = { -1, -1, -1 };

		//this is an axis aligned bounding box. that means it consists of 6 planes, 2 for every axis
		//the ray intersects with each plane, either inside or outside of the box
		//first we discard, for every axis, the intersection with the furthest plane (2 planes per axis -> 2 intersections per axis)

		if (ray.direction.x > 0)
		{
			rayIntersectionDistances.x = (minPos.x - ray.origin.x) / ray.direction.x; //this is the closest for x plane if x ray dir is positive
		}
		else if (ray.direction.x < 0)
		{
			rayIntersectionDistances.x = (maxPos.x - ray.origin.x) / ray.direction.x; //this is the closest if x ray dir is negative
		}

		if (ray.direction.y > 0)
		{
			rayIntersectionDistances.y = (minPos.y - ray.origin.y) / ray.direction.y; //this is the closest for y plane if y ray dir is positive
		}
		else if (ray.direction.y < 0)
		{
			rayIntersectionDistances.y = (maxPos.y - ray.origin.y) / ray.direction.y; //this is the closest if y ray dir is negative
		}

		if (ray.direction.z > 0)
		{
			rayIntersectionDistances.z = (minPos.z - ray.origin.z) / ray.direction.z; //this is the closest for z plane if z ray dir is positive
		}
		else if (ray.direction.z < 0)
		{
			rayIntersectionDistances.z = (maxPos.z - ray.origin.z) / ray.direction.z; //this is the closest if z ray dir is negative
		}

		//now we have 3 intersection distances with 3 planes
		//with these and the ray's origin and direction vectors we can get the 3 intersection points
		//but we don't need the 3, because we know the furthest point from those 3 is the only one that could be on the box's surface
		//how do we know it? hard to explain, if you draw a 2d box and a ray is easier to understand visually
		float possibleIntersectionDistance = Math::max(Math::max(
			rayIntersectionDistances.x,
			rayIntersectionDistances.y),
			rayIntersectionDistances.z);
		if (possibleIntersectionDistance < 0.f)
		{
			return false; //the box is behind the ray
		}
		Vector3 possibleIntersectionPoint = ray.origin + (ray.direction * possibleIntersectionDistance);

		constexpr float epsilon = Math::epsilon();
		if ((possibleIntersectionPoint.x + epsilon < minPos.x || possibleIntersectionPoint.x - epsilon > maxPos.x) ||
			(possibleIntersectionPoint.y + epsilon < minPos.y || possibleIntersectionPoint.y - epsilon > maxPos.y) ||
			(possibleIntersectionPoint.z + epsilon < minPos.z || possibleIntersectionPoint.z - epsilon > maxPos.z))
		{
			return false; //the intersection point is outside the box
		}

		hit.point = possibleIntersectionPoint;
		hit.distanceFromRayOrigin = possibleIntersectionDistance;
		return true;
	}

	bool ColliderFuncs::isPointInsideBox(const Vector3& point, const Vector3& halfSize)
	{
		return Math::abs(point.x) <= halfSize.x && Math::abs(point.y) <= halfSize.y && Math::abs(point.z) <= halfSize.z;
	}

	bool ColliderFuncs::testSeparatingAxis(Vector3 axis, const Vector3 box1Axes[3],
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

	bool ColliderFuncs::testAllSeparatingAxes(const Vector3 box1Axes[3], const Vector3 box2Axes[3], const Vector3& box1HalfSize,
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

	float ColliderFuncs::projectBox(const Vector3& axisToProject, const Vector3 boxAxes[3], const Vector3& boxHalfSize)
	{
		return	Math::abs(Vector3::dot(axisToProject, boxAxes[0]) * boxHalfSize.x) +
			Math::abs(Vector3::dot(axisToProject, boxAxes[1]) * boxHalfSize.y) +
			Math::abs(Vector3::dot(axisToProject, boxAxes[2]) * boxHalfSize.z);
	}

	void ColliderFuncs::getBoxVertices(const Vector3& position, const Vector3& halfSize, const Vector3 axes[3], Vector3 vertices[8])
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

	bool ColliderFuncs::isCollidingSphereBox(const Vector3& spherePosition, float sphereRadius, const Vector3& boxPosition,
		const Vector3& boxHalfSize, Collision& col)
	{
		Vector3 aabbMin = boxPosition - boxHalfSize;
		Vector3 aabbMax = boxPosition + boxHalfSize;
		Vector3 closestPoint = Vector3::clamp(spherePosition, aabbMin, aabbMax);
		Vector3 delta = closestPoint - spherePosition;
		float distSquared = Vector3::squaredMagnitude(delta);
		if (distSquared > sphereRadius * sphereRadius)
		{
			return false;
		}

		float distance = Math::sqrt(distSquared);

		if (distance > Math::epsilon())
		{
			col.normal = delta / distance;
			col.depth = sphereRadius - distance;
			col.contactPoints[0] = closestPoint;
		}
		else //sphere center is inside the box
		{
			Vector3 directionFromCenter = spherePosition - boxPosition;
			Vector3 absDirectionFromCenter = Vector3::abs(directionFromCenter);

			if (absDirectionFromCenter.x > absDirectionFromCenter.y && absDirectionFromCenter.x > absDirectionFromCenter.z)
			{
				col.normal = { directionFromCenter.x > 0.f ? 1.f : -1.f, 0, 0 };
			}
			else if (absDirectionFromCenter.y > absDirectionFromCenter.z)
			{
				col.normal = { 0, directionFromCenter.y > 0.f ? 1.f : -1.f, 0 };
			}
			else
			{
				col.normal = { 0, 0, directionFromCenter.z > 0.f ? 1.f : -1.f };
			}

			col.depth = sphereRadius;
			col.contactPoints[0] = spherePosition + col.normal * sphereRadius;
		}
		col.contactPointsCount = 1;
		return true;
	}

#pragma endregion


#pragma region INTERSECTION_COLLISION


	bool ColliderFuncs::isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit)
	{
		//this method does a vector space basis change to align the axes with the box roation and place the box at local 0 0 0.
		//doing this allows us to treat the box as an AABB and use it's intersection check function.
		//to avoid using 4x4 transform matrices we use 3x3s to perform the rotations of the basis change, and for the translation transform
		//we translate the ray origin by subtracting the box origin, and then adding it again to the hit point
		//scale is not supported so this way we do the same as 4x4 transform matrices with 3x3s and a vector 

		Vector3 position = reg.getComponent<Position>(entity).value;
		Quaternion rotation = reg.getComponent<Rotation>(entity).value;
		Matrix3x3 localToWorldSpaceTransform = Matrix3x3(rotation);
		Matrix3x3 worldToLocalSpaceTransform = Matrix3x3(Quaternion::conjugate(rotation));

		Vector3 localRayOrigin = ray.origin - position;

		Ray rayInLocalSpace = { worldToLocalSpaceTransform * localRayOrigin, worldToLocalSpaceTransform * ray.direction };
		bool intersected = ColliderFuncs::isIntersectingBox(rayInLocalSpace, Vector3::zero(), collider.halfSize, hit);

		if (intersected)
		{
			hit.point = (localToWorldSpaceTransform * hit.point) + position;
		}
		return intersected;
	}

	bool ColliderFuncs::isIntersectingSphere(const Registry& reg, const Ray& ray, const SphereCollider& collider,
		Entity entity, RayHit& hit)
	{
		Vector3 spherePos = reg.getComponent<Position>(entity).value;
		Vector3 direction = spherePos - ray.origin; //from ray origin to sphere center
		float projected = Vector3::dot(direction, ray.direction); //direction projected in the ray direction

		if (projected < 0.f)
		{
			return false; //the sphere is behind the ray
		}

		Vector3 closestPointInsideRay = ray.origin + (ray.direction * projected);

		float distanceToSphereCenter = Vector3::distance(closestPointInsideRay, spherePos);

		if (distanceToSphereCenter > collider.radius)
		{
			return false; //closest point is outside the sphere
		}

		float offset = Math::sqrt((collider.radius * collider.radius) - (distanceToSphereCenter * distanceToSphereCenter));
		//the 2 points are at ray distance = projected + -offset, the closest one is the smallest (-)
		hit.distanceFromRayOrigin = projected - offset;
		hit.point = ray.origin + (ray.direction * hit.distanceFromRayOrigin);
		//this method does not assign the entity to the RayHit struct
		return true;
	}

	bool ColliderFuncs::isIntersectingAABB(const Registry& reg, const Ray& ray, const AABBCollider& collider, Entity entity, RayHit& hit)
	{
		Position pos = reg.getComponent<Position>(entity);
		return isIntersectingBox(ray, pos.value, collider.halfSize, hit);
	}


	bool ColliderFuncs::isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2,
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
			if (ColliderFuncs::isPointInsideBox(localPoint, c2.halfSize))
			{
				contactPoints.push_back(vertex1);
			}
		}

		//check which vertices of the box 2 are inside the box 1
		for (const Vector3& vertex2 : vertices2)
		{
			//transform the vertex to the obb local space so it acts as an aabb
			Vector3 localPoint = Quaternion::rotate(inverseRotation1, vertex2 - position1);
			if (ColliderFuncs::isPointInsideBox(localPoint, c1.halfSize))
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

	bool ColliderFuncs::isCollidingSphereOBB(const Registry& reg, const SphereCollider& c1, const OBBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 spherePosition = reg.getComponent<Position>(e1).value;
		Vector3 obbPosition = reg.getComponent<Position>(e2).value;
		Quaternion obbRotation = reg.getComponent<Rotation>(e2).value;

		Matrix3x3 obbLocalToWorldSpaceTransform = Matrix3x3(obbRotation);
		Matrix3x3 worldToObbLocalSpaceTransform = Matrix3x3(Quaternion::conjugate(obbRotation));

		//sphere position in obb's local space
		Vector3 localSpherePos = worldToObbLocalSpaceTransform * (spherePosition - obbPosition);

		if (!ColliderFuncs::isCollidingSphereBox(localSpherePos, c1.radius, Vector3::zero(), c2.halfSize, col))
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


	bool ColliderFuncs::isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2,
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
			if (ColliderFuncs::isPointInsideBox(localPoint, c2.halfSize))
			{
				contactPoints.push_back(vertex1);
			}
		}

		//check which vertices of the box 2(aabb) are inside the box 1(obb)
		for (const Vector3& vertex2 : vertices2)
		{
			//transform the vertex to the obb local space so it acts as an aabb
			Vector3 localPoint = Quaternion::rotate(inverseRotation1, vertex2 - position1);
			if (ColliderFuncs::isPointInsideBox(localPoint, c1.halfSize))
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

	bool ColliderFuncs::isCollidingSphereSphere(const Registry& reg, const SphereCollider& c1, const SphereCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 pos1 = reg.getComponent<Position>(e1).value;
		Vector3 pos2 = reg.getComponent<Position>(e2).value;
		Vector3 delta = pos2 - pos1;
		float distanceSq = Vector3::squaredMagnitude(delta);
		float totalRadius = c1.radius + c2.radius;
		if (distanceSq >= totalRadius * totalRadius)
		{
			return false;
		}

		float distance = Math::sqrt(distanceSq);
		if (distance > Math::epsilon())
		{
			col.normal = delta / distance;
			Vector3 contactPoint1 = pos1 + col.normal * c1.radius;
			Vector3 contactPoint2 = pos2 - col.normal * c2.radius;
			col.contactPoints[0] = (contactPoint1 + contactPoint2) * .5f;
			col.depth = totalRadius - distance;
		}
		else //the spheres at almost entirely overlapped
		{
			col.normal = { 0, 1, 0 }; //arbitrary normal
			col.contactPoints[0] = (pos1 + pos2) * .5f;
			col.depth = totalRadius;

		}
		col.contactPointsCount = 1;
		col.entity1 = e1;
		col.entity2 = e2;
		return true;
	}

	bool ColliderFuncs::isCollidingSphereAABB(const Registry& reg, const SphereCollider& c1, const AABBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 spherePosition = reg.getComponent<Position>(e1).value;
		Vector3 aabbPosition = reg.getComponent<Position>(e2).value;

		if (!isCollidingSphereBox(spherePosition, c1.radius, aabbPosition, c2.halfSize, col))
		{
			return false;
		}
		col.entity1 = e1;
		col.entity2 = e2;
		return true;
	}

	bool ColliderFuncs::isCollidingAABBAABB(const Registry& reg, const AABBCollider& c1, const AABBCollider& c2,
		Entity e1, Entity e2, Collision& col)
	{

		Vector3 pos1 = reg.getComponent<Position>(e1).value;
		Vector3 pos2 = reg.getComponent<Position>(e2).value;
		Vector3 min1 = pos1 - c1.halfSize;
		Vector3 max1 = pos1 + c1.halfSize;
		Vector3 min2 = pos2 - c2.halfSize;
		Vector3 max2 = pos2 + c2.halfSize;


		Vector3 overlap = Vector3::min(max1, max2) - Vector3::max(min1, min2);

		if (overlap.x <= 0 || overlap.y <= 0 || overlap.z <= 0)
		{
			return false;
		}

		float minOverlap = overlap.x;
		col.normal = { 1, 0, 0 };
		if (overlap.y < minOverlap)
		{
			minOverlap = overlap.y;
			col.normal = { 0, 1, 0 };
		}
		if (overlap.z < minOverlap)
		{
			minOverlap = overlap.z;
			col.normal = { 0, 0, 1 };
		}

		Vector3 delta = pos2 - pos1;
		if (Vector3::dot(delta, col.normal) < 0)
		{
			col.normal = -col.normal; // Make sure normal points from c1 to c2
		}


		//col.contactPoint = (Vector3::max(min1, min2) + Vector3::min(max1, max2)) * .5f;

		col.depth = minOverlap;
		col.entity1 = e1;
		col.entity2 = e2;

		col.contactPointsCount = 4;



		// Contact box / volume intersection
		Vector3 contactMin = Vector3::max(min1, min2);
		Vector3 contactMax = Vector3::min(max1, max2);

		Vector3 contactCenter = (contactMin + contactMax) * 0.5f;

		// Contact box sides
		Vector3 size = contactMax - contactMin;

		// Select the face perpendicular to the normal (contact plane)
		// and get the 4 vertices
		if (Math::abs(col.normal.x) > 0.9f)
		{
			// Contact on X axis -> YZ face
			Vector3 y = { 0, size.y * 0.5f, 0 };
			Vector3 z = { 0, 0, size.z * 0.5f };
			Vector3 center = contactCenter;
			col.contactPoints[0] = center + y + z;
			col.contactPoints[1] = center + y - z;
			col.contactPoints[2] = center - y + z;
			col.contactPoints[3] = center - y - z;

		}
		else if (Math::abs(col.normal.y) > 0.9f)
		{
			// Contact on Y axis -> XZ face
			Vector3 x = { size.x * 0.5f, 0, 0 };
			Vector3 z = { 0, 0, size.z * 0.5f };
			Vector3 center = contactCenter;
			col.contactPoints[0] = center + x + z, col.depth;
			col.contactPoints[1] = center + x - z, col.depth;
			col.contactPoints[2] = center - x + z, col.depth;
			col.contactPoints[3] = center - x - z, col.depth;

		}
		else
		{
			// Contact on Z axis -> XY face
			Vector3 x = { size.x * 0.5f, 0, 0 };
			Vector3 y = { 0, size.y * 0.5f, 0 };
			Vector3 center = contactCenter;
			col.contactPoints[0] = center + x + y, col.depth;
			col.contactPoints[1] = center + x - y, col.depth;
			col.contactPoints[2] = center - x + y, col.depth;
			col.contactPoints[3] = center - x - y, col.depth;
		}

		return true;
	}

#pragma endregion



#pragma region INERTIA_TENSOR

	Matrix3x3 ColliderFuncs::calculateBoxInverseInertiaTensor(const Vector3& halfSize, float inverseMass)
	{
		Vector3 size = 2.f * halfSize;
		float sx = std::max(size.x, Math::epsilon());
		float sy = std::max(size.y, Math::epsilon());
		float sz = std::max(size.z, Math::epsilon());
		Vector3 diagonal = Vector3
		{
			(12.f * inverseMass) / (sz * sz + sy * sy),
			(12.f * inverseMass) / (sx * sx + sz * sz),
			(12.f * inverseMass) / (sx * sx + sy * sy)
		};
		return Matrix3x3(diagonal);
	}
	
	Matrix3x3 ColliderFuncs::calculateRotatedBoxInverseInertiaTensor(const Matrix3x3& localInvInertia, const Quaternion& rotation)
	{
		Matrix3x3 rotationMatrix(rotation);
		Matrix3x3 rotationMatrixTransposed = Matrix3x3::transpose(rotationMatrix);
		return rotationMatrix * localInvInertia * rotationMatrixTransposed;
	}
	
	Matrix3x3 ColliderFuncs::calculateSphereInverseInertiaTensor(float radius, float inverseMass)
	{
		float diagonal = (5 * inverseMass) / (2 * radius * radius);
		return Matrix3x3(Vector3{ diagonal, diagonal, diagonal });
	}

#pragma endregion

#pragma region BOUNDS
	void ColliderFuncs::calculateSphereBounds(float radius, ColliderBounds& bounds)
	{
		radius += BOUNDS_PADDING;
		bounds.localMin = Vector3{ -radius, -radius, -radius };
		bounds.localMax = Vector3{ radius, radius, radius };
	}

	void ColliderFuncs::calculateBoxBounds(Vector3 halfSize, ColliderBounds& bounds)
	{
		//first we make a sphere that fully contains the box, no matter it's rotation
		//then we get that sphere's bounds
		float enclosingSphereRadius = Vector3::magnitude(halfSize);
		return calculateSphereBounds(enclosingSphereRadius, bounds);
	}
#pragma endregion
}