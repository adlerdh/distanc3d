#pragma once

#include "IPriorityQueue.h"
#include <fiboqueue.h>

/// @see https://github.com/beniz/fiboheap
template<class T, class Comparer, class Hasher>
class FibonacciQueue : public IPriorityQueue<T>
{
public:
  FibonacciQueue() = default;
  ~FibonacciQueue() = default;

  void clear() override { m_queue.clear(); }
  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override { return m_queue.top(); }
  void push(const T& item) override { m_queue.push(item); }
  void pop() override { m_queue.pop(); }

  constexpr bool mutableKeys() override { return true; }

  void decreaseKey(const T& item) override
  {
    if (typename FibHeap<T, Comparer>::FibNode* x = m_queue.findNode(item))
    {
      m_queue.decrease_key(x, item);
    }
  }

private:
  mutable FibQueue<T, Comparer, Hasher> m_queue;
};
