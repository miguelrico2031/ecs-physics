#pragma once
#include "Entity.h"
#include <vector>
#include <optional>
#include <cassert>
#include <tuple>

namespace epl
{

	// Interface for storing ComponentPool pointers
	class IComponentPool
	{
	public:
		virtual ~IComponentPool() = default;
		virtual void removeIfPresent(Entity entity) = 0;
	};

	template <class T, Entity MAX_COMPONENTS>
	class ComponentPool : public IComponentPool
	{

	public:
		ComponentPool() = default;
		~ComponentPool() override = default;

		template <typename ...Args>
		T& add(Entity entity, Args&&... args)
		{
			static_assert(std::is_constructible_v<T, Args...>, "Component cannot be constructed with the given arguments.");
			assert(entity <= MAX_COMPONENTS && "Entity out of bounds.");
			assert(!m_storage[entity].has_value() && "Cannot add same component twice.");

			m_highestEntityEver = std::max(m_highestEntityEver, entity);

			m_storage[entity].emplace(std::forward<Args>(args)...);
			return m_storage[entity].value();
		}

		void remove(Entity entity)
		{
			assert(m_storage[entity].has_value() && "Cannot remove non existent component.");
			m_storage[entity].reset();
		}

		bool has(Entity entity) const
		{
			return m_storage[entity].has_value();
		}

		T& get(Entity entity)
		{
			return *m_storage[entity];
		}

		const T& get(Entity entity) const
		{
			return *m_storage[entity];
		}

		std::optional<T>& getOptional(Entity entity)
		{
			return m_storage[entity];
		}

		const std::optional<const T>& getOptional(Entity entity) const
		{
			return m_storage[entity];
		}

		void removeIfPresent(Entity entity) override
		{
			if (has(entity))
			{
				remove(entity);
			}
		}

	private:
		std::vector<std::optional<T>> m_storage = std::vector<std::optional<T>>(MAX_COMPONENTS);
		Entity m_highestEntityEver = 0;

#pragma region Iterators

	public:
		class Iterator
		{
		public:
			Iterator(ComponentPool<T, MAX_COMPONENTS>& pool, Entity index) : m_pool(pool), m_currentEntity(index) {}
			
			bool operator!=(const Iterator& other) const
			{
				return m_currentEntity != other.m_currentEntity;
			}

			void operator++()
			{
				do
				{
					m_currentEntity++;
				}
				while (m_currentEntity <= m_pool.m_highestEntityEver && !m_pool.m_storage[m_currentEntity].has_value());
			}
			std::tuple<Entity, T&> operator*()
			{
				return { m_currentEntity, *m_pool.m_storage[m_currentEntity] };
			}
		private:
			ComponentPool<T, MAX_COMPONENTS>& m_pool;
			Entity m_currentEntity;
		};

		class ConstIterator
		{
		public:
			ConstIterator(const ComponentPool<T, MAX_COMPONENTS>& pool, Entity index) : m_pool(pool), m_currentEntity(index) {}

			bool operator!=(const ConstIterator& other) const
			{
				return m_currentEntity != other.m_currentEntity;
			}

			void operator++()
			{
				do
				{
					m_currentEntity++;
				} while (m_currentEntity <= m_pool.m_highestEntityEver && !m_pool.m_storage[m_currentEntity].has_value());
			}
			std::tuple<Entity, const T&> operator*()
			{
				return { m_currentEntity, *m_pool.m_storage[m_currentEntity] };
			}
		private:
			const ComponentPool<T, MAX_COMPONENTS>& m_pool;
			Entity m_currentEntity;
		};
		
#pragma endregion


		Iterator begin() { return Iterator(*this, 0); }
		Iterator end() { return Iterator(*this, m_highestEntityEver + 1); }
		ConstIterator begin() const { return ConstIterator(*this, 0); }
		ConstIterator end() const { return ConstIterator(*this, m_highestEntityEver + 1); }
	};
}