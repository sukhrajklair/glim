#include <glim/localization/prior_map/prior_map.hpp>

namespace glim {

PriorMap::PriorMap() : loaded(false) {}

bool PriorMap::load(const std::string& path) {
  source_path = path;
  loaded = !source_path.empty();
  return loaded;
}

bool PriorMap::is_loaded() const {
  return loaded;
}

}  // namespace glim
