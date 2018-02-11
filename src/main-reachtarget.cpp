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

#define CUSTOM

#ifdef CUSTOM
#define MAP_OUTPUT_FILENAME "map.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 0.4
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"
#define TG_SIZE 64
#define START_POS_X 0
#define START_POS_Y -4
#define START_YAW 0
#define TARGET_POS_X -4
#define TARGET_POS_Y 4
#define TARGET_YAW (2.0 * M_PI_2)
#define RRT_GROWTH_FACTOR 1
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 5000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.9
#endif

#ifdef ROCKS
#define MAP_OUTPUT_FILENAME "map.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/arc_rocks.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 0.4
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_arc_rocks_full.dot"
#define TG_SIZE 64
#define START_POS_X -3
#define START_POS_Y -3
#define START_YAW M_PI_2
#define TARGET_POS_X -3
#define TARGET_POS_Y 3
#define TARGET_YAW M_PI_2
#define RRT_GROWTH_FACTOR 1
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 5000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.9
#endif


void build_map( VRepClient& client, const HeightMap& map ){
    client.create_heightfield_shape(
            map.size_x_px(), map.size_y_px(), map.size_x_mt(),
            map.size_z_mt(), map.get_data() );
}

void write_svg_of_map_and_plan( const HeightMap& map, const ReachTargetTask* task ){
    svg::SVGWriter svg_out( MAP_OUTPUT_FILENAME );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_plan( svg_out, task->get_plan() );
    svg::utils::write_path( svg_out, task->get_path() );
    svg_out.end();
}

int main( int argc, char *argv[] ) {
    std::cout << "Connecting to V-rep server...";
    VRepClient client( SERVER_ADDRESS, SERVER_PORT );
    client.connect();
    std::cout << (client.is_connected() ? "ok\n" : "failed\n");

    // Simulation parameters
    HeightMap map( MAP_FILENAME, MAP_X_METERS, MAP_HEIGHT );
    map.load_traversability_graph( TG_FILENAME, TG_SIZE, TG_SIZE );
    Point3D start_position( START_POS_X, START_POS_Y, 0.1 );
    Point3D start_orientation( 0, 0, START_YAW );
    Point2D start_position_2d( start_position.x(), start_position.y() );
    double start_yaw = start_orientation.z();
    Point2D target_position_2d( TARGET_POS_X, TARGET_POS_Y );
    double target_yaw = TARGET_YAW;

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
        task->setRRTParameters( RRT_GROWTH_FACTOR, RRT_GREEDYNESS,
                                RRT_MAX_ITERATIONS, RRT_MAX_SEGMENT_ANGLE,
                                RRT_TRAVERSABILITY_THRESHOLD );
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
