#pragma once

#include <glim/odometry/estimation_frame.hpp>

namespace glim {

class KeyframeSelector {
public:
  KeyframeSelector();

  bool is_keyframe(const EstimationFrame::ConstPtr& frame);

private:
  long last_keyframe_id;
};

}  // namespace glim
