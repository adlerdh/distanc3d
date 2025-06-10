#pragma once

#include "IPriorityQueue.h"
#include <updatable_priority_queue.h>

/// @see https://github.com/Ten0/updatable_priority_queue
template<class T, bool MutableKeys = true>
class UpdatablePriorityQueue : public IPriorityQueue<T>
{
public:
  using PQType
    = better_priority_queue::updatable_priority_queue<typename T::first_type, typename T::second_type>;

  UpdatablePriorityQueue() = default;
  ~UpdatablePriorityQueue() = default;

  void clear() override { m_queue = PQType(); }
  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override
  {
    m_lastTop.first = m_queue.top().key;
    m_lastTop.second = m_queue.top().priority;
    return m_lastTop;
  }

  void push(const T& item) override { m_queue.push(item.first, -item.second); }
  void pop() override { m_queue.pop(); }

  constexpr bool mutableKeys() override { return MutableKeys; }
  void changeKey(const T& item) override { m_queue.update(item.first, item.second); }

private:
  PQType m_queue;
  mutable T m_lastTop;
};
