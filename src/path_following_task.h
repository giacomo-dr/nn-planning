// path_following_task.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef PATH_FOLLOWING_TASK_H
#define PATH_FOLLOWING_TASK_H


#include "robot_task.h"
#include "manta_controller.h"
#include "path_follower.h"


class PathFollowingTask : public RobotTask {
public:
    PathFollowingTask( MantaController& manta, WaypointPath2D path );

    virtual CallResult initialize( int loop_delay_ms );
    virtual void finalize();
    virtual CallResult controlStep( long time_now );
    virtual void abort();

private:
    MantaController& manta;
    PIDPathFollower path_follower;
    int loop_delay_ms;
};


#endif //PATH_FOLLOWING_TASK_H
