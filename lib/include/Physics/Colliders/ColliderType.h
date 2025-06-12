#pragma once

#ifndef MAX_COLLIDER_TYPES
#define MAX_COLLIDER_TYPES 16
#endif

#include <cassert>

using ColliderTypeID = size_t;

namespace epl
{
	class ColliderType
	{
	public:
		template<class Collider_T>
		static ColliderTypeID getColliderTypeID()
		{
			static ColliderTypeID id = s_nextID++;
			assert(id < MAX_COLLIDER_TYPES && "Too many collider types used."
				"To be able to use more types, do #define MAX_COLLIDER_TYPES <new number> before any #include of this file.");
			return id;
		}

		static ColliderTypeID getColliderTypeCount() { return s_nextID; }
	private:
		static ColliderTypeID s_nextID;
	};


}