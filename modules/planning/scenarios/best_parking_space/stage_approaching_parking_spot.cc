#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/best_parking_space/stage_approaching_parking_spot.h"

namespace apollo {
namespace planning {

// 初始化函数
bool StageBestApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir,
    void* context) {
    if (!Stage::Init(config, injector, config_dir, context)) {
    return false;  // 初始化父类失败，返回 false
    }
    scenario_config_.CopyFrom(GetContextAs<BestParkingSpaceContext>()->scenario_config);
    return true;
}

// 处理函数
StageResult StageBestApproachingParkingSpot::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageBestApproachingParkingSpot";
    CHECK_NOTNULL(frame);  // 检查 frame 是否为 nullptr
    StageResult result;
    auto scenario_context = GetContextAs<BestParkingSpaceContext>();

    if (scenario_context->target_parking_spot_id.empty()) {
    return result.SetStageStatus(StageStatusType::ERROR);  // 目标停车位 ID 为空，返回错误状态
    }

    // 设置目标停车位信息
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;
    frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(scenario_context->pre_stop_rightaway_flag);
    *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) = scenario_context->pre_stop_rightaway_point;

    //gs_change







    //

    // 处理参考线上的障碍物决策
    auto* reference_lines = frame->mutable_reference_line_info();
    for (auto& reference_line : *reference_lines) 
    {
        auto* path_decision = reference_line.path_decision();
        if (nullptr == path_decision) 
        {
            continue;
        }
        auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);
        if (nullptr == dest_obstacle) 
        {
            continue;
        }
        ObjectDecisionType decision;
        decision.mutable_ignore();  // 忽略目标障碍物
        dest_obstacle->EraseDecision();
        dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking", decision);
    }
    result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

    // 更新 scenario_context
    scenario_context->pre_stop_rightaway_flag = frame->open_space_info().pre_stop_rightaway_flag();
    scenario_context->pre_stop_rightaway_point = frame->open_space_info().pre_stop_rightaway_point();

    if (CheckADCStop(*frame)) {
    next_stage_ = "BEST_PARKING_PARKING";  // 进入下一个阶段
    return StageResult(StageStatusType::FINISHED);
    }
    if (result.HasError()) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);  // 持续运行
}

// 检查车辆是否停止
bool StageBestApproachingParkingSpot::CheckADCStop(const Frame& frame) {
    const auto& reference_line_info = frame.reference_line_info().front();
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;  // 速度过快，未停止
    }

    // 检查车辆是否距离停止线足够近
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double stop_fence_start_s = frame.open_space_info().open_space_pre_stop_fence_s();
    const double distance_stop_line_to_adc_front_edge = stop_fence_start_s - adc_front_edge_s;

    if (distance_stop_line_to_adc_front_edge > scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;  // 车辆距离停止线太远
    }
    return true;  // 车辆已停止并且距离合适
}

}  // namespace planning
}  // namespace apollo
