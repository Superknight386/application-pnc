#include <memory>
#include "modules/planning/traffic_rules/speed_bump_speed_limit/speed_bump_speed_limit.h"

namespace apollo {
namespace planning {

/* 定义成员函数*/

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool SpeedBumpSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    // Load the config this task.
    return TrafficRule::LoadConfig<SpeedBumpSpeedLimitConfig>(&config_);
}

Status SpeedBumpSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
    const std::vector<PathOverlap>& speed_bump_overlaps
            = reference_line_info->reference_line().map_path().speed_bump_overlaps();
    for (const auto& speed_bump_overlap : speed_bump_overlaps) {
        AINFO<<"speed_bump_id:"<<speed_bump_overlap.object_id;
        reference_line->AddSpeedLimit(
                speed_bump_overlap.start_s - config_.forward_buffer(),
                speed_bump_overlap.end_s + config_.backward_buffer(),
                config_.limit_speed());
    }
    return Status::OK();
}

}  // namespace planning
}  // namespace apollo