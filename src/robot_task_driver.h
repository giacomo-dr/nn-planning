// robot_task_driver.h
// Author: Giacomo Del Rio
// Date: 29 August 2017

#ifndef ROBOT_TASK_DRIVER_H
#define ROBOT_TASK_DRIVER_H

#include <iostream>
#include <csignal>
#include "vrep_client.h"
#include "robot_task.h"


class RobotTaskDriver
{
public:
    RobotTaskDriver( VRepClient& client, int loop_delay_ms );
    virtual ~RobotTaskDriver(){};
    bool execute( RobotTask* task );

private:
    void control_loop();
    static void sigint_handler( int sig );

private:
    VRepClient& client;        // VRep client proxy
    int loop_delay_ms;         // Interval between two consecutive executions of the control loop
    RobotTask* task;           // The task to be executed

    // SIGINT flag
    static volatile std::sig_atomic_t termination_signal;
    // ms of sleep after the call to task->initialize() and the begin of control loop
    static const int post_initialize_sleep_ms;
    // ms of sleep after the exiting the control loop
    static const int final_sleep_ms;
};

#endif // ROBOT_TASK_DRIVER_H
