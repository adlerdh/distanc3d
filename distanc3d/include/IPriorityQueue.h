#pragma once

#include <cstdlib>

/**
 * @brief Interface for a priority queue, based on std::priority_queue, that may have mutable keys.
 * Allows for interchange of different priority queue implementations into Dijkstra's method.
 *
 * @see https://en.cppreference.com/w/cpp/container/priority_queue
 */
template<class T>
class IPriorityQueue
{
public:
  ~IPriorityQueue() = default;

  virtual void clear() = 0;
  virtual bool empty() const = 0;
  virtual std::size_t size() const = 0;

  virtual const T& top() const = 0;
  virtual void push(const T&) = 0;
  virtual void pop() = 0;

  constexpr virtual bool mutableKeys() = 0;

  // These need only be implemented when mutableKeys() returns true
  virtual void changeKey(const T&) {}
  virtual void decreaseKey(const T&) {}
  virtual void increaseKey(const T&) {}
};
