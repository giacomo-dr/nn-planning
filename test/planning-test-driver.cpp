// planning-test-driver.cpp
// Author: Giacomo Del Rio
// Date: 28 Feb 2018

#include <iostream>
#include "rrt_star_planner.h"
#include "svg_utils.h"

using std::string;

struct PlanningProblem{
    string name;
    string map_filename;
    string tg_filename;
    double map_x_meters;
    double map_height;
    int tg_size;
    Point2D start_position;
    double start_yaw;
    Point2D target_position;
    double  target_yaw;
    double growth_factor;
    double max_segment_angle;
};

struct PlanningSolver{
    string name;
    string class_name;
    unsigned int greedyness;
    unsigned int max_iterations;
    double traversability_threshold;
    int grow_to_target_neighbors;
    int grow_to_point_neighbors;
};

const string heightmaps_folder = "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/";
const string tgraph_folder = "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/";

std::vector<PlanningProblem> problems{
        {.name = "Custom",
             .map_filename = "custom9.png",
             .tg_filename = "t_graph_cnn_custom9_full.dot",
             .map_x_meters = 10.0,
             .map_height = 0.4,
             .tg_size = 64,
             .start_position = Point2D( 0, -4 ),
             .start_yaw = 0,
             .target_position = Point2D( -4, 4 ),
             .target_yaw = 2.0 * M_PI_2,
             .growth_factor = 0.3,
             .max_segment_angle = M_PI / 6.0},
        {.name = "Rocks",
            .map_filename = "arc_rocks.png",
            .tg_filename = "t_graph_cnn_arc_rocks_full.dot",
            .map_x_meters = 10.0,
            .map_height = 1,
            .tg_size = 64,
            .start_position = Point2D( 4, -4 ),
            .start_yaw = M_PI,
            .target_position = Point2D( 3, 4.5 ),
            .target_yaw = 0,
            .growth_factor = 0.3,
            .max_segment_angle = M_PI / 6.0
        }
};

std::vector<PlanningSolver> solvers{
        {.name = "RRT",
            .class_name = "RRTPlanner",
            .greedyness = 10,
            .max_iterations = 10000,
            .traversability_threshold = 0.95,
            .grow_to_target_neighbors = 10,
            .grow_to_point_neighbors = 1
        }
};


#define QUARRY

#ifdef CUSTOM
#define MAP_OUTPUT_FILENAME "plan_custom.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 0.4
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"
#define TG_SIZE 64
#define START_POS_X 0
#define START_POS_Y -4
#define START_POS_Z 0.2
#define START_YAW 0
#define TARGET_POS_X -4
#define TARGET_POS_Y 4
#define TARGET_YAW (2.0 * M_PI_2)
#define RRT_GROWTH_FACTOR 0.3
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 12000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.9
#define PF_PID_LIN_PROPORTIONAL_GAIN 2.5
#define PF_PID_LIN_INTEGRAL_GAIN 2
#define PF_PID_LIN_DERIVATIVE_GAIN 0
#define PF_PID_ANG_PROPORTIONAL_GAIN 0.8
#define PF_PID_ANG_INTEGRAL_GAIN 0
#define PF_PID_ANG_DERIVATIVE_GAIN 0
#define PF_MAX_LINEAR_VEL 10.0
#define PF_MAX_LINEAR_ACC 100.0
#define PF_MAX_ANGULAR_VEL 1.1
#define PF_MAX_ANGULAR_ACC 100.0
#define PF_PATH_BLENDING 0.1
#define PF_INPLACE_ROTATION_THRESHOLD 100
#define PF_ANGLE_TOLERANCE 0.02
#define PF_ANTI_LOOP true
#define PF_ANTI_LOOP_RADIUS (0.3 * RRT_GROWTH_FACTOR / (2 * std::tan(RRT_MAX_SEGMENT_ANGLE / 2.0)))
#endif

#ifdef ROCKS
#define MAP_OUTPUT_FILENAME "plan_rocks.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/arc_rocks.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 1
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_arc_rocks_full.dot"
#define TG_SIZE 64
#define START_POS_X 4
#define START_POS_Y -4
#define START_POS_Z 0.4
#define START_YAW M_PI
#define TARGET_POS_X 3
#define TARGET_POS_Y 4.5
#define TARGET_YAW 0
#define RRT_GROWTH_FACTOR 0.3
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 14000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.95
#define PF_PID_LIN_PROPORTIONAL_GAIN 4.0
#define PF_PID_LIN_INTEGRAL_GAIN 2
#define PF_PID_LIN_DERIVATIVE_GAIN 0.0
#define PF_PID_ANG_PROPORTIONAL_GAIN 1.5
#define PF_PID_ANG_INTEGRAL_GAIN 0.0
#define PF_PID_ANG_DERIVATIVE_GAIN 0.0
#define PF_MAX_LINEAR_VEL 20.0
#define PF_MAX_LINEAR_ACC 100.0
#define PF_MAX_ANGULAR_VEL 2.2
#define PF_MAX_ANGULAR_ACC 100.0
#define PF_PATH_BLENDING 0.1
#define PF_ANGLE_TOLERANCE 0.02
#define PF_INPLACE_ROTATION_THRESHOLD 100
#define PF_ANTI_LOOP true
#define PF_ANTI_LOOP_RADIUS (0.3 * RRT_GROWTH_FACTOR / (2 * std::tan(RRT_MAX_SEGMENT_ANGLE / 2.0)))
#endif

#ifdef ELEVATION
#define MAP_OUTPUT_FILENAME "plan_elevation.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/gridmap_elevation_2_c_r.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 1
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_gridmap_elevation_2_c_r.dot"
#define TG_SIZE 59
#define START_POS_X 0
#define START_POS_Y -1
#define START_POS_Z 0.2
#define START_YAW 0
#define TARGET_POS_X 2
#define TARGET_POS_Y 3
#define TARGET_YAW M_PI_2
#define RRT_GROWTH_FACTOR 0.3
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 5000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.6
#define PF_PID_LIN_PROPORTIONAL_GAIN 2.5
#define PF_PID_LIN_INTEGRAL_GAIN 2
#define PF_PID_LIN_DERIVATIVE_GAIN 0
#define PF_PID_ANG_PROPORTIONAL_GAIN 0.8
#define PF_PID_ANG_INTEGRAL_GAIN 0
#define PF_PID_ANG_DERIVATIVE_GAIN 0
#define PF_MAX_LINEAR_VEL 10.0
#define PF_MAX_LINEAR_ACC 100.0
#define PF_MAX_ANGULAR_VEL 1.1
#define PF_MAX_ANGULAR_ACC 100.0
#define PF_PATH_BLENDING 0.1
#define PF_INPLACE_ROTATION_THRESHOLD 100
#define PF_ANGLE_TOLERANCE 0.02
#define PF_ANTI_LOOP true
#define PF_ANTI_LOOP_RADIUS (0.3 * RRT_GROWTH_FACTOR / (2 * std::tan(RRT_MAX_SEGMENT_ANGLE / 2.0)))
#endif

#ifdef HEIGHTMAP
#define MAP_OUTPUT_FILENAME "plan_heightmap.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/heightmap1.png"
#define MAP_X_METERS 10.0
#define MAP_HEIGHT 0.4
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_heightmap1_full.dot"
#define TG_SIZE 64
#define START_POS_X -4
#define START_POS_Y -4
#define START_POS_Z 0.2
#define START_YAW M_PI_2
#define TARGET_POS_X -4
#define TARGET_POS_Y 4
#define TARGET_YAW M_PI
#define RRT_GROWTH_FACTOR 0.3
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 8000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 7.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.8
#define PF_PID_LIN_PROPORTIONAL_GAIN 4
#define PF_PID_LIN_INTEGRAL_GAIN 2
#define PF_PID_LIN_DERIVATIVE_GAIN 0
#define PF_PID_ANG_PROPORTIONAL_GAIN 0.8
#define PF_PID_ANG_INTEGRAL_GAIN 0
#define PF_PID_ANG_DERIVATIVE_GAIN 0
#define PF_MAX_LINEAR_VEL 10.0
#define PF_MAX_LINEAR_ACC 100.0
#define PF_MAX_ANGULAR_VEL 1.1
#define PF_MAX_ANGULAR_ACC 100.0
#define PF_PATH_BLENDING 0.1
#define PF_INPLACE_ROTATION_THRESHOLD 100
#define PF_ANGLE_TOLERANCE 0.02
#define PF_ANTI_LOOP true
#define PF_ANTI_LOOP_RADIUS (0.3 * RRT_GROWTH_FACTOR / (2 * std::tan(RRT_MAX_SEGMENT_ANGLE / 2.0)))
#endif

#ifdef QUARRY
#define MAP_OUTPUT_FILENAME "plan_quarry.svg"
#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/quarry_cropped4_scaled2cm.png"
#define MAP_X_METERS 30.0
#define MAP_HEIGHT 8
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_quarry_stride_14.dot"
#define TG_SIZE 110
#define DO_NOT_LOAD_MAP
#define START_POS_X 7
#define START_POS_Y -1.0
#define START_POS_Z 4
#define START_YAW M_PI_2
#define TARGET_POS_X -10.5
#define TARGET_POS_Y 9.27
#define TARGET_YAW M_PI
#define RRT_GROWTH_FACTOR 0.3
#define RRT_GREEDYNESS 10
#define RRT_MAX_ITERATIONS 60000
#define RRT_MAX_SEGMENT_ANGLE (M_PI / 6.0)
#define RRT_TRAVERSABILITY_THRESHOLD 0.9
#define PF_PID_LIN_PROPORTIONAL_GAIN 6
#define PF_PID_LIN_INTEGRAL_GAIN 4
#define PF_PID_LIN_DERIVATIVE_GAIN 0
#define PF_PID_ANG_PROPORTIONAL_GAIN 0.8
#define PF_PID_ANG_INTEGRAL_GAIN 0
#define PF_PID_ANG_DERIVATIVE_GAIN 0
#define PF_MAX_LINEAR_VEL 10.0
#define PF_MAX_LINEAR_ACC 100.0
#define PF_MAX_ANGULAR_VEL 1.1
#define PF_MAX_ANGULAR_ACC 100.0
#define PF_PATH_BLENDING 0.1
#define PF_INPLACE_ROTATION_THRESHOLD 100
#define PF_ANGLE_TOLERANCE 0.02
#define PF_ANTI_LOOP true
#define PF_ANTI_LOOP_RADIUS (0.5 * RRT_GROWTH_FACTOR / (2 * std::tan(RRT_MAX_SEGMENT_ANGLE / 2.0)))
#endif


void write_svg_of_map_and_plan( const HeightMap& map, const RRTPlanner& planner ){
    svg::SVGWriter svg_out( MAP_OUTPUT_FILENAME );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_plan( svg_out, planner );
    svg::utils::write_path( svg_out, planner.get_path() );
    svg_out.end();
}

void write_svg_of_map_and_plan( const HeightMap& map, const RRTStarPlanner& planner ){
    svg::SVGWriter svg_out( MAP_OUTPUT_FILENAME );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_star_plan( svg_out, planner );
    svg::utils::write_path( svg_out, planner.get_path() );
    svg_out.end();
}

int main( int argc, char *argv[] ) {

    // Simulation parameters
//    std::cout << "Load map...\n";
//    HeightMap map( MAP_FILENAME, MAP_X_METERS, MAP_HEIGHT );
//    map.load_traversability_graph( TG_FILENAME, TG_SIZE, TG_SIZE );
//    Point3D start_position( START_POS_X, START_POS_Y, START_POS_Z );
//    Point3D start_orientation( 0, 0, START_YAW );
//    Point2D start_position_2d( start_position.x(), start_position.y() );
//    double start_yaw = start_orientation.z();
//    Point2D target_position_2d( TARGET_POS_X, TARGET_POS_Y );
//    double target_yaw = TARGET_YAW;
//    std::cout << "ok\n";
//
//    std::cout << "Build plan...\n";
//    RRTPlanner planner( &map, RRT_GROWTH_FACTOR, RRT_GREEDYNESS,
//                        RRT_MAX_ITERATIONS, RRT_MAX_SEGMENT_ANGLE,
//                        RRT_TRAVERSABILITY_THRESHOLD );
//    planner.build_plan( start_position_2d, start_yaw, target_position_2d, target_yaw );
//    std::cout << "ok\n";
//
//    write_svg_of_map_and_plan( map, planner );

    // For each problem
    for( PlanningProblem p: problems ){
        std::cout << "Loading test case '" << p.name << "'\n";
        HeightMap map( heightmaps_folder + p.map_filename, p.map_x_meters, p.map_height );
        map.load_traversability_graph( tgraph_folder + p.tg_filename, p.tg_size, p.tg_size );
        // For each solver
        for( PlanningSolver s: solvers ){
            std::cout << "\tSolving with '" << s.name << "'\n";
            RRTPlanner::Parameters params = {
                    .growth_factor = p.growth_factor,
                    .max_segment_angle = p.max_segment_angle,
                    .greediness = s.greedyness,
                    .max_iterations = s.max_iterations,
                    .traversability_threshold = s.traversability_threshold,
                    .grow_to_target_neighbors = s.grow_to_target_neighbors,
                    .grow_to_point_neighbors = s.grow_to_point_neighbors
            };
            RRTPlanner planner( &map, params );
            int res = planner.build_plan( p.start_position, p.start_yaw,
                                          p.target_position, p.target_yaw );
            std::cout << (res == -1 ? "\t\tTarget not reached" : "Target reached") << std::endl;
        }
    }
    return 0;
}
