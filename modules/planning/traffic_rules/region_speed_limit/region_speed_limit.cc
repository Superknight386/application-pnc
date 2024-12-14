#include <memory>
 #include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"
 
 namespace apollo {
 namespace planning {
 
 /* 定义成员函数*/
 
 using apollo::common::Status;
 using apollo::hdmap::PathOverlap;
 
 bool RegionSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
     if (!TrafficRule::Init(name, injector)) {
         return false;
     }
     // Load the config this task.
     return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
 }
 
 Status RegionSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
     ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
     const std::vector<PathOverlap>& junction_overlaps
             = reference_line_info->reference_line().map_path().junction_overlaps();
     for (const auto& junction_overlap : junction_overlaps) {
         reference_line->AddSpeedLimit(
                 junction_overlap.start_s - config_.forward_buffer(),
                 junction_overlap.end_s + config_.backward_buffer(),
                 config_.limit_speed());
     }
     return Status::OK();
 }
 
 }  // namespace planning
 }  // namespace apollo