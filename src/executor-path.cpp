// executor.cpp
// Author: Giacomo Del Rio
// Date: 10 Apr 2017

#include <iostream>
#include <csignal>
#include "manta_controller.h"
#include "path_following_task.h"


#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 19997
#define LOOP_DELAY_MS 100


// Catch Ctr-C to proper cleanup
volatile std::sig_atomic_t termination_signal = 0;
void sigint_handler( int sig ){ termination_signal = 1; }

void control_loop( VRepClient& client, RobotTask* task ){
    task->initialize( LOOP_DELAY_MS );
    client.sleep( 100 ); // Initial sleep

    while( client.is_connected() ){
        if( termination_signal ){
            task->abort();
            client.sleep( 100 ); // Final sleep
            return;
        }

        int now = client.get_last_cmd_time();
        int res = task->controlStep( now );
        if( res != RES_OK )
            break; // Task completed or error occurred
        client.sleep( LOOP_DELAY_MS );
    }

    if( client.is_connected() ){
        task->finalize();
        client.sleep(100); // Final sleep
    }
}

WaypointPath2D make_path(){
    WaypointPath2D path;
    path.waypoints.push_back( Point2D(0, 0) );
    path.waypoints.push_back( Point2D(4, 0) );
    path.waypoints.push_back( Point2D(4, 4) );
    path.waypoints.push_back( Point2D(0, 4) );
    return path;
}

int main( int argc, char *argv[] ) {
    std::signal( SIGINT, sigint_handler );

    std::cout << "Connecting to default V-rep server...";
    VRepClient client( SERVER_ADDRESS, SERVER_PORT );
    client.connect();
    std::cout << (client.is_connected() ? "ok\n" : "failed\n");

    if( client.is_connected() ){
        MantaController manta( client );
        WaypointPath2D path = make_path();
        RobotTask* task = new PathFollowingTask( manta, path );
        control_loop( client, task );
        delete task;
        client.disconnect();
    }

    return 0;
}
