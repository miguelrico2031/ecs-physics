#pragma once
#include <ECS/ComponentPool.h>
#include <queue>
#include <memory>


namespace epl
{
	class Registry
	{
	public:
		Registry(Entity maxEntities) : m_maxEntities(maxEntities)
		{
			assert(maxEntities < NULL_ENTITY && "MAX_ENTITIES must be less than 0xFFFFFFFF");
		}

		~Registry() = default;

		size_t getMaxEntities() const { return m_maxEntities; }

		// Creates and returns a new empty entity.
		Entity createEntity()
		{
			assert(m_aliveEntityCount <= m_maxEntities && "Maximum number of entities reached.");

			m_aliveEntityCount++;
			if (m_freeEntities.empty())
			{
				return m_nextEntity++;
			}
			else
			{
				Entity entity = m_freeEntities.front();
				m_freeEntities.pop();
				return entity;
			}
		}

		// Destroys the entity and all its components.
		void destroyEntity(Entity entity)
		{
			for (auto& pool : m_componentPools)
			{
				pool->removeIfPresent(entity);
			}

			m_freeEntities.push(entity);
			m_aliveEntityCount--;
		}

		// Registers a new component type. Use this when initializing the Registry to add custom component types.
		// Adding a component type before registering it will throw an assertion failure.
		template<class Component_T>
		void registerComponentType()
		{
			size_t index = getIndex<Component_T>();
			assert(index >= m_componentPools.size() && "Component type already registered.");
			m_componentPools.push_back(std::make_unique<ComponentPool<Component_T>>(m_maxEntities));
		}

		template <class Component_T, typename... Args>
		Component_T& addComponent(Entity entity, Args&&... args)
		{
			auto* pool = poolPtr<Component_T>();
			return pool->add(entity, std::forward<Args>(args)...);
		}

		template <class Component_T, typename... Args>
		Component_T& addOrSetComponent(Entity entity, Args&&... args)
		{
			auto* pool = poolPtr<Component_T>();
			return pool->has(entity)
				? pool->set(entity, std::forward<Args>(args)...)
				: pool->add(entity, std::forward<Args>(args)...);
		}

		template<class Component_T>
		void removeComponent(Entity entity)
		{
			auto* pool = poolPtr<Component_T>();
			pool->remove(entity);
		}

		template<class Component_T>
		bool hasComponent(Entity entity) const
		{
			auto* pool = poolPtr<Component_T>();
			return pool->has(entity);
		}

		template<class Component_T>
		Component_T& getComponent(Entity entity)
		{
			auto* pool = poolPtr<Component_T>();
			return pool->get(entity);
		}

		template<class Component_T>
		const Component_T& getComponent(Entity entity) const
		{
			auto* pool = poolPtr<Component_T>();
			return pool->get(entity);
		}

		template<class Component_T>
		std::optional<Component_T&> tryGetComponent(Entity entity)
		{
			auto* pool = poolPtr<Component_T>();
			return pool->getOptional(entity);
		}

		template<class Component_T>
		const std::optional<const Component_T&> tryGetComponent(Entity entity) const
		{
			auto* pool = poolPtr<Component_T>();
			return pool->getOptional(entity);
		}


		template<class Component_T>
		ComponentPool<Component_T>& iterate() { return *poolPtr<Component_T>(); }

		template<class Component_T>
		const ComponentPool<Component_T>& iterate() const { return *poolPtr<Component_T>(); }

	private:
		template<class Component_T>
		//returns a unique index for each component type
		size_t getIndex() const
		{
			static size_t idx = m_componentTypeCount++;
			return idx;
		}


		//Helper functions to get the casted pointer to a component pool knowing only the component type.
		template<class Component_T>
		ComponentPool<Component_T>* poolPtr() const
		{
			size_t index = getIndex<Component_T>();
			assert(index < m_componentPools.size() && "Component type not registered.");
			auto* pool = poolPtr<Component_T>(m_componentPools[index]);
			assert(pool != nullptr && "Component pool is null.");
			return pool;
		}

		template<class Component_T>
		ComponentPool<Component_T>* poolPtr(const std::unique_ptr<IComponentPool>& pool) const
		{
			return static_cast<ComponentPool<Component_T>*>(pool.get());
		}


	private:
		const Entity m_maxEntities;
		Entity m_nextEntity = 0;
		Entity m_aliveEntityCount = 0;
		std::queue<Entity> m_freeEntities;

		mutable size_t m_componentTypeCount = 0;
		std::vector<std::unique_ptr<IComponentPool>> m_componentPools;
	};
}