#pragma once

#include "IPriorityQueue.h"
#include <jkds/container/priority_queue.h>
#include <vector>

/// @see https://github.com/jkomyno/jkds
template<class T, bool MutableKeys = true>
class JkdsPriorityQueue : public IPriorityQueue<T>
{
public:
  using KeyType = typename T::second_type;
  using ValueType = typename T::first_type;

  JkdsPriorityQueue()
    : m_queue(std::vector<KeyType>{}, std::vector<ValueType>{})
  {
  }

  ~JkdsPriorityQueue() = default;

  void clear() override
  {
    while (!m_queue.empty())
    {
      m_queue.pop();
    }
  }

  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override
  {
    const auto& topKeyValue = m_queue.top_key_value();
    m_lastTop.first = topKeyValue.second;
    m_lastTop.second = topKeyValue.first;
    return m_lastTop;
  }

  void push(const T& item) override { m_queue.push(item.second, item.first); }
  void pop() override { m_queue.pop(); }

  constexpr bool mutableKeys() override { return MutableKeys; }
  void changeKey(const T& item) override { m_queue.update_key(item.second, item.first); }
  void decreaseKey(const T& item) override { changeKey(item); }
  void increaseKey(const T& item) override { changeKey(item); }

  bool contains(const T& item) { return m_queue.contains(item.first); }

private:
  static constexpr bool isHeap = false;
  static constexpr std::size_t K = 3;
  static constexpr auto HeapType = jkds::container::detail::heap_type::min_heap;
  using KHeapType = jkds::container::KHeap<HeapType, K, ValueType, isHeap>;

  jkds::container::PriorityQueue<HeapType, KHeapType, KeyType, ValueType> m_queue;
  mutable T m_lastTop;
};
