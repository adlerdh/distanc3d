#pragma once

#include "IPriorityQueue.h"
#include <queue>

template<class T, class Comparer>
class StdPriorityQueue : public IPriorityQueue<T>
{
  using QueueType = std::priority_queue<T, std::vector<T>, Comparer>;

public:
  StdPriorityQueue() = default;
  ~StdPriorityQueue() = default;

  void clear() override { m_queue = QueueType(); }
  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override { return m_queue.top(); }
  void push(const T& item) override { m_queue.push(item); }
  void pop() override { m_queue.pop(); }

  constexpr bool mutableKeys() override { return false; }

private:
  QueueType m_queue;
};
