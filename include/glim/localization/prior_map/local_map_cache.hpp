#pragma once

#include <deque>
#include <memory>

namespace glim {

class LocalMapCache {
public:
  LocalMapCache();

  void clear();
  std::size_t size() const;

private:
  std::deque<int> cached_keys;
};

}  // namespace glim
