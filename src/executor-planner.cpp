// executor.cpp
// Author: Giacomo Del Rio
// Date: 10 Apr 2017

#include <iostream>
#include <csignal>
#include "manta_controller.h"
#include "reach_target_task.h"
#include "svg_utils.h"


#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 19997
#define LOOP_DELAY_MS 100

#define MAP_FILENAME "/icloud/Data/UniversitaMaster/Thesis/simpleheight.tiff"

// Catch Ctr-C to proper cleanup
volatile std::sig_atomic_t termination_signal = 0;
void sigint_handler( int sig ){ termination_signal = 1; }

void control_loop( VRepClient& client, RobotTask* task ){
    if( task->initialize( LOOP_DELAY_MS ) == RES_OK ){
        client.sleep( 100 ); // Initial sleep

        while( client.is_connected() ){
            if( termination_signal ){
                task->abort();
                client.sleep( 100 ); // Final sleep
                return;
            }

            long now = client.get_last_cmd_time();
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
}

void build_map( VRepClient& client, const HeightMap& map ){
    client.create_heightfield_shape(
            map.size_x_px(), map.size_y_px(), map.size_x_mt(),
            map.size_z_mt(), map.get_data() );
}

void write_svg_of_map_and_plan( const HeightMap& map, const ReachTargetTask* task ){
    svg::SVGWriter svg_out( "map.svg" );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_plan( svg_out, task->get_plan() );
    svg::utils::write_path( svg_out, task->get_path() );
    svg_out.end();
}

int main( int argc, char *argv[] ) {
    std::signal( SIGINT, sigint_handler );

    std::cout << "Connecting to default V-rep server...";
    VRepClient client( SERVER_ADDRESS, SERVER_PORT );
    client.connect();
    std::cout << (client.is_connected() ? "ok\n" : "failed\n");

    // Simulation parameters
    HeightMap map( MAP_FILENAME, 10, 0.4 );
    Point3D start_position( 0, -4, 0.3 );
    Point3D start_orientation( 0, 0, 0 );
    Point2D start_position_2d( start_position.x(), start_position.y() );
    double start_yaw = start_orientation.z();
    Point2D target_position_2d( 3, 4.5 );
    double target_yaw = 0;

    if( client.is_connected() ){
        client.start_simulation();
        client.sleep( 100 );
        MantaController manta( client );
        build_map( client, map );
        manta.set_pose( start_position, start_orientation );
        client.sleep( 100 );
        ReachTargetTask* task = new ReachTargetTask(
                manta, map, start_position_2d, start_yaw,
                target_position_2d, target_yaw );
        control_loop( client, task );
        write_svg_of_map_and_plan( map, task );
        delete task;
        client.stop_simulation();
        client.sleep( 100 );
        client.disconnect();
    }

    return 0;
}

/*
    // wheel radius:         0.09
    // wheel base:             0.6
    // wheel track:             0.35
    // maximum steering rate:     70 deg/sec

    // the maximum steer angle 30 degree
    max_steer_angle=0.5235987
    // the maximum torque of the motor
    motor_torque=60
*/