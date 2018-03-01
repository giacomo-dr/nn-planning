// reach_target_task.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef REACH_TARGET_TASK_H
#define REACH_TARGET_TASK_H


#include "robot_task.h"
#include "manta_controller.h"
#include "path_following_task.h"
#include "rrt_planner.h"
#include "svg_writer.h"


class ReachTargetTask : public RobotTask {
public:
    ReachTargetTask( MantaController& manta, HeightMap& map,
                     Point2D start_pos, double start_yaw,
                     Point2D target_pos, double target_yaw );
    void setRRTParameters( const RRTPlanner::Parameters& params );
    void setFollowerParameters( const PIDPathFollower::Parameters& params );
    CallResult initialize( int loop_delay_ms ) override;
    void finalize() override;
    CallResult controlStep( long time_now ) override;
    void abort() override;

    const RRTPlan& get_plan() const;
    const WaypointPath2D& get_path() const;

private:
    MantaController& manta;
    std::unique_ptr<PathFollowingTask> follow_task;
    RRTPlanner rrt_planner;
    HeightMap& map;
    Point2D start_pos, target_pos;
    double start_yaw, target_yaw;
    PIDPathFollower::Parameters follower_params;
};


#endif //REACH_TARGET_TASK_H
