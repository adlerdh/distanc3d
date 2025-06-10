#pragma once

#include "IPriorityQueue.h"
#include <gheap_cpp11.hpp>
#include <gpriority_queue.hpp>

/// @see https://github.com/valyala/gheap
template<class T, class Comparer>
class GPriorityQueue : public IPriorityQueue<T>
{
public:
  GPriorityQueue()
    : m_queue(std::begin(m_container), std::end(m_container))
  {
  }

  ~GPriorityQueue() = default;

  void clear() override { m_container.clear(); }
  bool empty() const override { return m_queue.empty(); }
  std::size_t size() const override { return m_queue.size(); }

  const T& top() const override { return m_queue.top(); }
  void push(const T& item) override { m_queue.push(item); }
  void pop() override { m_queue.pop(); }

  constexpr bool mutableKeys() override { return false; }

private:
  static constexpr std::size_t Fanout = 2;
  static constexpr std::size_t PageChunks = 1;

  using Heap = gheap<Fanout, PageChunks>;
  using Container = std::vector<T>;
  using PriorityQueue = gpriority_queue<Heap, T, Container, Comparer>;

  Container m_container;
  PriorityQueue m_queue;
};
