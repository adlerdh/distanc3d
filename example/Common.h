#pragma once

#include <ostream>
#include <streambuf>

/// Greater-than comparator that provides a strict weak ordering on priority queue items
template<typename T>
struct GreaterThanComparer
{
  bool operator()(const std::pair<int, T>& a, const std::pair<int, T>& b) const
  {
    return a.second > b.second;
  }
};

class NullBuffer : public std::streambuf
{
public:
  int overflow(int c) override { return c; }
  std::streamsize xsputn(const char*, std::streamsize n) override { return n; }
};

class NullOstream : public std::ostream
{
public:
  NullOstream() : std::ostream(&m_buffer) {}

private:
  NullBuffer m_buffer;
};
