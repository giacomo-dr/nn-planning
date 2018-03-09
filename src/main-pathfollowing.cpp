// executor.cpp
// Author: Giacomo Del Rio
// Date: 10 Apr 2017


#include <iostream>
#include "manta_controller.h"
#include "path_following_task.h"
#include "height_map.h"
#include "robot_task_driver.h"


#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 19997
#define LOOP_DELAY_MS 100
#define MAP_FILENAME "/icloud/Data/UniversitaMaster/Thesis/simpleheight.tiff"
//#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 0.4
#define START_POS_X 0
#define START_POS_Y 0
#define START_YAW 0


void build_map( VRepClient& client, const HeightMap& map ){
    client.create_heightfield_shape(
            map.size_x_px(), map.size_y_px(), map.size_x_mt(),
            map.size_z_mt(), map.get_data() );
}

int main( int argc, char *argv[] ) {
    std::cout << "Connecting to V-rep server...";
    VRepClient client( SERVER_ADDRESS, SERVER_PORT );
    client.connect();
    std::cout << (client.is_connected() ? "ok\n" : "failed\n");

    // Simulation parameters
    HeightMap map( MAP_FILENAME, MAP_X_METERS, MAP_HEIGHT );
    Point3D start_position( START_POS_X, START_POS_Y, 0 );
    Point3D start_orientation( 0, 0, START_YAW );
    WaypointPath2D path;
    path.waypoints.push_back( Point2D(0, 0) );
    path.waypoints.push_back( Point2D(4, 0) );
    path.waypoints.push_back( Point2D(4, -4) );
    path.waypoints.push_back( Point2D(-4, -4) );
    path.waypoints.push_back( Point2D(-4, 0) );
    path.waypoints.push_back( Point2D(0, 0) );

    if( client.is_connected() ){
        // Prepare simulation
        client.start_simulation();
        client.sleep( 100 );
        MantaController manta( client );
        build_map( client, map );
        manta.set_pose( start_position, start_orientation );
        client.sleep( 100 );

        // Execute task
        PathFollowingTask* task = new PathFollowingTask( manta, path );
        task->setFollowerParameters( PIDPathFollower::Parameters{
                .linProportionalGain = 2,
                .linIntegralGain = 1,
                .linDerivativeGain = 0,
                .angProportionalGain = 0.8,
                .angIntegralGain = 0,
                .angDerivativeGain = 0,
                .maxLinVel = 8,
                .maxAngVel = 1.1,
                .maxLinAcc = 100,
                .maxAngAcc = 100,
                .pathBlending = 0.1,
                .angleTolerance = 0.02,
                .inPlaceRotationThreshold = 100,
                .antiLoop = true,
                .antiLoopRadius = 0.17
        });
        RobotTaskDriver driver( client, LOOP_DELAY_MS );
        driver.execute( task );

        // Post execution duties and cleanup
        delete task;
        client.stop_simulation();
        client.sleep( 100 );
        client.disconnect();
    }

    return 0;
}