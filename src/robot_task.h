// robot_task.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

enum CallResult{ RES_OK = 0, RES_COMPLETED = 1, RES_FAIL = -1 };

class RobotTask {
public:
    virtual ~RobotTask(){};
    virtual CallResult initialize( int loop_delay_ms ) = 0;
    virtual void finalize() = 0;
    virtual CallResult controlStep( long time_now ) = 0;
    virtual void abort() = 0;
};

#endif //ROBOT_TASK_H
