// robot_task_driver.cpp
// Author: Giacomo Del Rio
// Date: 29 August 2017


#include "robot_task_driver.h"

const int RobotTaskDriver::post_initialize_sleep_ms = 100;
const int RobotTaskDriver::final_sleep_ms = 100;
volatile std::sig_atomic_t RobotTaskDriver::termination_signal = 0;

RobotTaskDriver::RobotTaskDriver( VRepClient& client, int loop_delay_ms )
        : client(client){
    this->loop_delay_ms = loop_delay_ms;
}

bool RobotTaskDriver::execute( RobotTask* task ){
    this->task = task;

    if( task->initialize( loop_delay_ms ) == RES_OK ){
        termination_signal = 0;
        std::signal( SIGINT, RobotTaskDriver::sigint_handler );
        client.sleep( post_initialize_sleep_ms );
        control_loop();
        std::signal( SIGINT, SIG_DFL );
        return true;
    }else{
        return false;
    }
}

void RobotTaskDriver::control_loop(){
    // Control loop
    while( client.is_connected() ){
        // Handle termination by SIGINT
        if( termination_signal ){
            task->abort();
            client.sleep( final_sleep_ms );
            return;
        }

        long now = client.get_last_cmd_time();
        int res = task->controlStep( now );
        if( res != RES_OK )
            break; // Task completed or error occurred
        client.sleep( loop_delay_ms );
    }

    // Finalization
    if( client.is_connected() ){
        task->finalize();
        client.sleep( final_sleep_ms );
    }
}

void RobotTaskDriver::sigint_handler( int sig ){
    termination_signal = 1;
}

