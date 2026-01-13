#include <glim/localization/keyframe_selector.hpp>

namespace glim {

KeyframeSelector::KeyframeSelector() : last_keyframe_id(-1) {}

bool KeyframeSelector::is_keyframe(const EstimationFrame::ConstPtr& frame) {
  if (!frame) {
    return false;
  }

  if (last_keyframe_id < 0) {
    last_keyframe_id = frame->id;
    return true;
  }

  if (frame->id != last_keyframe_id) {
    last_keyframe_id = frame->id;
    return true;
  }

  return false;
}

}  // namespace glim
