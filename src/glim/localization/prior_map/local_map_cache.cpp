#include <glim/localization/prior_map/local_map_cache.hpp>

namespace glim {

LocalMapCache::LocalMapCache() = default;

void LocalMapCache::clear() {
  cached_keys.clear();
}

std::size_t LocalMapCache::size() const {
  return cached_keys.size();
}

}  // namespace glim
