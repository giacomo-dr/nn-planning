// executor.cpp
// Author: Giacomo Del Rio
// Date: 10 Apr 2017

#include <iostream>
#include "manta_controller.h"
#include "reach_target_task.h"
#include "robot_task_driver.h"
#include "svg_utils.h"


#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 19997
#define LOOP_DELAY_MS 100
// #define MAP_FILENAME "/icloud/Data/UniversitaMaster/Thesis/simpleheight.tiff"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"


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
    std::cout << "Connecting to default V-rep server...";
    VRepClient client( SERVER_ADDRESS, SERVER_PORT );
    client.connect();
    std::cout << (client.is_connected() ? "ok\n" : "failed\n");

    // Simulation parameters
    HeightMap map( MAP_FILENAME, 10, 0.4 );
    Point3D start_position( 0, -4, 0 );
    Point3D start_orientation( 0, 0, 0 );
    Point2D start_position_2d( start_position.x(), start_position.y() );
    double start_yaw = start_orientation.z();
    Point2D target_position_2d( -4, 0 );
    double target_yaw = 3.0 * M_PI_2;

    if( client.is_connected() ){
        // Prepare simulation
        client.start_simulation();
        client.sleep( 100 );
        MantaController manta( client );
        build_map( client, map );
        manta.set_pose( start_position, start_orientation );
        client.sleep( 100 );

        // Execute task
        ReachTargetTask* task = new ReachTargetTask(
                manta, map, start_position_2d, start_yaw,
                target_position_2d, target_yaw );
        RobotTaskDriver driver( client, LOOP_DELAY_MS );
        driver.execute( task );

        // Post execution duties and cleanup
        write_svg_of_map_and_plan( map, task );
        delete task;
        client.stop_simulation();
        client.sleep( 100 );
        client.disconnect();
    }

    return 0;
}
