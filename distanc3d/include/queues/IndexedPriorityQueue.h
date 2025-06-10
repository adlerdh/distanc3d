#pragma once

#include "IPriorityQueue.h"
#include "IndexedPQ.h"

/// @see https://github.com/kartikkukreja/blog-codes/blob/master/src/Indexed%20Min%20Priority%20Queue.cpp
template<class T, bool MutableKeys = true>
class MyIndexedPriorityQueue : public IPriorityQueue<T>
{
public:
  MyIndexedPriorityQueue(int maxSize)
    : m_queue(maxSize)
  {
  }

  ~MyIndexedPriorityQueue() = default;

  void clear() override { m_queue.clear(); }
  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override
  {
    m_lastTop.first = m_queue.minItem();
    m_lastTop.second = m_queue.minKey();
    return m_lastTop;
  }

  void push(const T& item) override { m_queue.insert(item.first, item.second); }
  void pop() override { m_queue.deleteMin(); }

  constexpr bool mutableKeys() override { return MutableKeys; }
  void changeKey(const T& item) override { m_queue.changeKey(item.first, item.second); }
  void decreaseKey(const T& item) override { m_queue.decreaseKey(item.first, item.second); }
  void increaseKey(const T& item) override { m_queue.increaseKey(item.first, item.second); }

private:
  IndexedPriorityQueue<typename T::second_type> m_queue;
  mutable T m_lastTop;
};
