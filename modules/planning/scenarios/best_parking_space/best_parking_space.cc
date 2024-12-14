/******************************************************************************
 * @file best_parking_space.cc
 *****************************************************************************/

#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"

#include "modules/planning/planning_base/common/frame.h"

// #include "modules/planning/scenarios/best_parking_space/stage_approaching_parking_spot.h"
// #include "modules/planning/scenarios/best_parking_space/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

// 初始化最佳停车位场景
bool BestParkingSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        // 如果已经初始化过，直接返回 true
        return true;
    }

    // 调用基类的初始化函数，如果失败则返回 false
    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    // 加载场景配置文件，如果失败则返回 false
    if (!Scenario::LoadConfig<BestParkingSpaceConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }

    // 获取高清地图指针，检查是否为空
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);

    // 初始化标志位设为 true
    init_ = true;
    return true;
}

// 判断场景是否可以转换
bool BestParkingSpaceScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    std::string target_parking_spot_id;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();

    // 检查是否有停车命令
    auto parking_command = frame.local_view().planning_command->has_parking_command();
    // 检查停车命令中是否包含停车位 ID
    auto parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();

    // 如果上一个场景为空或者参考线信息为空，则返回 false
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;

        if (parking_command && parking_spot_id) {
            // 获取目标停车位 ID
            target_parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();
        }
    }
    int state_flag = 2565127;
    const auto& vehicle_state = frame.vehicle_state();
    if(vehicle_state.y()<state_flag)
    {
        return false;
    }
    if (!parking_command) {
        // 获取路由终点信息
        const auto routing_end = frame.local_view().end_lane_way_point;
        common::PointENU end_postion;
        end_postion.set_x(routing_end->pose().x());
        end_postion.set_y(routing_end->pose().y());

        // 如果路由终点为空，返回 false
        if (nullptr == routing_end) {
            return false;
        }

        common::SLPoint dest_sl;
        const auto& reference_line_info = frame.reference_line_info().front();
        const auto& reference_line = reference_line_info.reference_line();

        // 将路由终点的 XY 坐标转换为 SL 坐标
        reference_line.XYToSL(routing_end->pose(), &dest_sl);

        // 获取车辆前沿在参考线上的 s 值
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        // 计算车辆到目的地的距离
        const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;

        // 如果距离大于设置的启动范围，则返回 false
        if (adc_distance_to_dest > parking_spot_range_to_start) {
            return false;
        }

        // 如果目标停车位 ID 为空，则从高清地图中获取附近的停车位
        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;

            // 在指定范围内获取停车位列表
            if (hdmap_->GetParkingSpaces(end_postion, 25, &parking_spaces) == 0
                && parking_spaces.size() > 0) {
                    // park_time[3]=Clock::NowInSeconds()
                    // std::string blocking_obstacle_id = "";
                    // const auto& reference_line_info = frame.reference_line_info().back();
                    // blocking_obstacle_id = reference_line_info.GetBlockingObstacle()->Id();
                    std::reverse(parking_spaces.begin(),parking_spaces.end());
                    for (auto parking_space:parking_spaces)
                    {
                        AINFO<<"parking_space_id:"<<parking_space->parking_space().id().id();
                    }
                    const auto obstacles = frame.reference_line_info().front().path_decision().obstacles().Items();
                    for(auto parking_space:parking_spaces)
                    {
                        int flag=1;
                        for(auto obstacle : obstacles)
                        {
                            if(obstacle->Id()=="DEST")
                                continue;
                            const auto obstacle_polygon = obstacle->PerceptionPolygon().points();
                            const auto parking_spot = parking_space->parking_space().polygon().point();
                            auto obstacle_polygon_y = (obstacle_polygon[0].y()+obstacle_polygon[3].y())/2;
                            auto obstacle_polygon_x = (obstacle_polygon[1].x()+obstacle_polygon[3].x())/2;
                            if(obstacle_polygon_y>parking_spot[0].y()&&obstacle_polygon_y<parking_spot[1].y())
                            {
                                if(obstacle_polygon_x>parking_spot[1].x()&&obstacle_polygon_x<parking_spot[3].x())
                                {
                                    flag=0;
                                    break;    
                                }
                              
                            }
                        }
                        if(flag==1)
                        {
                            target_parking_spot_id = parking_space->parking_space().id().id();
                            break;
                        }
                    }
                    
                    // if(ids[0]=="1000"&&ids[1]=="1001")
                    // {
                        // target_parking_spot_id = parking_spaces[2]->parking_space().id().id();//停入一号车位
                    // }
                    // std::string blocking_obstacle_id = "";
                    // for (const auto& obstacle : frame.obstacles())
                    // {
                    //     blocking_obstacle_id = obstacle->Id();
                    //     AINFO << "blocking_obstacle_id" << blocking_obstacle_id;
                    // }
                    // target_parking_spot_id = parking_spaces[2]->parking_space().id().id();//停入一号车位
                    // AINFO << "target_parking_spot_id" << target_parking_spot_id;
                // for (auto parking_space : parking_spaces) {
                //     // 获取第一个停车位的 ID 作为目标停车位 ID
                //     target_parking_spot_id = parking_space->parking_space().id().id();
                // }
            }
        }
    }

    // 如果仍未找到目标停车位 ID，返回 false
    if (target_parking_spot_id.empty()) {
        return false;
    }

    AINFO << "target_parking_spot_id" << target_parking_spot_id;

    // 获取参考线的地图路径
    const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
    PathOverlap parking_space_overlap;
    // const auto& vehicle_state = frame.vehicle_state();

    // 在路径上搜索目标停车位
    if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id, &parking_space_overlap)) {
        ADEBUG << "在所有可能的路径上未找到停车位: " << target_parking_spot_id;
        return false;
    }
    // 检查车辆与停车位的距离是否在范围内
    if (!CheckDistanceToParkingSpot(
            frame, vehicle_state, nearby_path, parking_spot_range_to_start, parking_space_overlap)) {
        ADEBUG << "找到目标停车位，但距离过远，超过预定义距离: " << target_parking_spot_id;
        return false;
    }
    // 设置目标停车位 ID
    context_.target_parking_spot_id = target_parking_spot_id;
    return true;
}

// 在路径上搜索目标停车位
bool BestParkingSpaceScenario::SearchTargetParkingSpotOnPath(
        const Path& nearby_path,
        const std::string& target_parking_id,
        PathOverlap* parking_space_overlap) {
    // 获取路径上的所有停车位重叠信息
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto& parking_overlap : parking_space_overlaps) {
        // 如果找到目标停车位，返回 true
        if (parking_overlap.object_id == target_parking_id) {
            *parking_space_overlap = parking_overlap;
            return true;
        }
    }
    // 未找到目标停车位，返回 false
    return false;
}

// 检查车辆与停车位的距离
bool BestParkingSpaceScenario::CheckDistanceToParkingSpot(
        const Frame& frame,
        const VehicleState& vehicle_state,
        const Path& nearby_path,
        const double parking_start_range,
        const PathOverlap& parking_space_overlap) {
    // TODO(Jinyun) 停车位重叠的 s 值在地图上不正确，无法使用
    // 获取高清地图指针
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id id;
    double center_point_s, center_point_l;
    id.set_id(parking_space_overlap.object_id);
    // 根据 ID 获取停车位信息
    ParkingSpaceInfoConstPtr target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
    // 获取停车位的四个顶点坐标
    Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
    Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
    Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
    // 计算停车位的中心点
    Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;
    // 获取中心点在路径上的最近投影点的 s 和 l 值
    nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    // 获取车辆当前位置的坐标
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    // 获取车辆位置在路径上的最近投影点的 s 和 l 值
    nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    // 判断车辆与停车位中心点的 s 值差距是否小于启动范围
    if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo

