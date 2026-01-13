#pragma once

#include <string>
#include <memory>

namespace glim {

class PriorMap {
public:
  using Ptr = std::shared_ptr<PriorMap>;

  PriorMap();

  bool load(const std::string& path);
  bool is_loaded() const;

private:
  bool loaded;
  std::string source_path;
};

}  // namespace glim
