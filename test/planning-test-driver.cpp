// planning-test-driver.cpp
// Author: Giacomo Del Rio
// Date: 28 Feb 2018

#include <iostream>
#include "rrt_star_planner.h"
#include "svg_utils.h"


#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


using std::string;

struct PlanningProblem{
    string name;
    string map_filename;
    string tg_filename;
    double map_x_meters;
    double map_height;
    int tg_size;
    double tg_padding;
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
    double neighbors_factor;
    RRTStarPlanner::OptMetric opt_metric;
    double threshold_step;
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
                .tg_padding = 0.6,
                .start_position = Point2D( 0, -4 ),
                .start_yaw = 0,
                .target_position = Point2D( -4, 4 ),
                .target_yaw = 2.0 * M_PI_2,
                .growth_factor = 0.3,
                .max_segment_angle = M_PI / 6.0
        },
        {.name = "Rocks",
                .map_filename = "arc_rocks.png",
                .tg_filename = "t_graph_cnn_arc_rocks_full.dot",
                .map_x_meters = 10.0,
                .map_height = 1,
                .tg_size = 64,
                .tg_padding = 0.3,
                .start_position = Point2D( 4, -4 ),
                .start_yaw = M_PI,
                .target_position = Point2D( 3, 4.5 ),
                .target_yaw = 0,
                .growth_factor = 0.3,
                .max_segment_angle = M_PI / 6.0
        },
        {.name = "Elevation",
                .map_filename = "gridmap_elevation_2_c_r.png",
                .tg_filename = "t_graph_cnn_gridmap_elevation_2_c_r.dot",
                .map_x_meters = 10.0,
                .map_height = 1,
                .tg_size = 59,
                .tg_padding = 0.3,
                .start_position = Point2D( 0, -1 ),
                .start_yaw = 0,
                .target_position = Point2D( 2, 3 ),
                .target_yaw = M_PI_2,
                .growth_factor = 0.3,
                .max_segment_angle = M_PI / 6.0
        },
        {.name = "Heightmap",
                .map_filename = "heightmap1.png",
                .tg_filename = "t_graph_cnn_heightmap1_full.dot",
                .map_x_meters = 10.0,
                .map_height = 0.4,
                .tg_size = 64,
                .tg_padding = 0.3,
                .start_position = Point2D( -4, -4 ),
                .start_yaw = M_PI_2,
                .target_position = Point2D( -4, 4 ),
                .target_yaw = M_PI,
                .growth_factor = 0.3,
                .max_segment_angle = M_PI / 6.0
        },
        {.name = "Quarry",
                .map_filename = "quarry_cropped4_scaled2cm.png",
                .tg_filename = "t_graph_cnn_quarry_stride_14.dot",
                .map_x_meters = 30.0,
                .map_height = 8,
                .tg_size = 110,
                .tg_padding = 0.3,
                .start_position = Point2D( 7, -1 ),
                .start_yaw = M_PI_2,
                .target_position = Point2D( -10.5, 9.27 ),
                .target_yaw = M_PI,
                .growth_factor = 0.3,
                .max_segment_angle = M_PI / 6.0
        }
};

std::vector<PlanningSolver> solvers{
//        {.name = "RRT",
//                .class_name = "RRTPlanner",
//                .greedyness = 10,
//                .max_iterations = 80000,
//                .traversability_threshold = 0.9,
//                .grow_to_point_neighbors = 20,
//                .opt_metric = RRTStarPlanner::OptMetric::distance,  // Unused
//                .threshold_step = 0                                 // Unused
//        },
        {.name = "RRTStar",
                .class_name = "RRTStarPlanner",
                .greedyness = 10,
                .max_iterations = 1200000,
                .traversability_threshold = 0.9,
                .grow_to_point_neighbors = 20,
                .opt_metric = RRTStarPlanner::OptMetric::distance,
                .threshold_step = 0
        },
        {.name = "RRTStar",
                .class_name = "RRTStarPlanner",
                .greedyness = 10,
                .max_iterations = 1200000,
                .traversability_threshold = 0.9,
                .grow_to_point_neighbors = 20,
                .opt_metric = RRTStarPlanner::OptMetric::probability,
                .threshold_step = 0
        }
};


void write_svg_of_map_and_plan( string filename, const HeightMap& map, const RRTPlanner& planner ){
    svg::SVGWriter svg_out( filename );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_plan( svg_out, planner );
    svg::utils::write_path( svg_out, planner.get_path() );
    svg_out.end();
}

void write_svg_of_map_and_plan( string filename, const HeightMap& map, const RRTStarPlanner& planner ){
    svg::SVGWriter svg_out( filename );
    svg::utils::initialize_svg_writer( svg_out, map );
    svg_out.begin();
    svg::utils::write_height_map( svg_out, map );
    svg::utils::write_rrt_star_plan( svg_out, planner );
    svg::utils::write_path( svg_out, planner.get_path() );
    svg_out.end();
}

string build_filename( const PlanningProblem &p, const PlanningSolver &s ){
    std::ostringstream res;
    res << p.name << "-" << s.name << "_" << (s.max_iterations / 1000);
    res << "k_tt" << s.traversability_threshold;
    if( s.grow_to_point_neighbors > -1 )
        res << "_ne" << s.grow_to_point_neighbors;
    switch( s.opt_metric ){
        case RRTStarPlanner::OptMetric::distance: res << "_dst"; break;
        case RRTStarPlanner::OptMetric::probability: res << "_prb"; break;
        case RRTStarPlanner::OptMetric::tr_threshold_stepping: res << "_stp"; break;
    }
    res << ".svg";
    return res.str();
}

int main( int argc, char *argv[] ) {

    // For each problem
    for( PlanningProblem p: problems ){
        if( argc > 1 && string(argv[1]) != "--first" && string(argv[1]) != p.name )
            continue;

        std::cout << "Loading test case '" << p.name << "'\n";
        HeightMap map( heightmaps_folder + p.map_filename, p.map_x_meters, p.map_height );
        map.load_traversability_graph( tgraph_folder + p.tg_filename, p.tg_size, p.tg_size,
                                       p.tg_padding, p.tg_padding );
        // For each solver
        for( PlanningSolver s: solvers ){
            std::cout << "\tSolving with '" << s.name << "'\n";

            if( s.class_name == "RRTPlanner" ){
                RRTPlanner::Parameters params = {
                        .growth_factor = p.growth_factor,
                        .max_segment_angle = p.max_segment_angle,
                        .greediness = s.greedyness,
                        .max_iterations = s.max_iterations,
                        .traversability_threshold = s.traversability_threshold,
                        .grow_to_point_neighbors = s.grow_to_point_neighbors
                };
                RRTPlanner planner( &map, params );
                int res = planner.build_plan( p.start_position, p.start_yaw,
                                              p.target_position, p.target_yaw );
                if( res == -1 )
                    std::cout << RED << "\t\tTarget not reached" << RESET << std::endl;
                else
                    std::cout << GREEN << "\t\tTarget reached" << RESET << std::endl;
                string out_file = build_filename( p, s );
                std::cout << "\t\tResult saved in " << out_file << std::endl;
                write_svg_of_map_and_plan( out_file, map, planner );
            }else if( s.class_name == "RRTStarPlanner" ){
                RRTStarPlanner::Parameters params = {
                        .growth_factor = p.growth_factor,
                        .max_segment_angle = p.max_segment_angle,
                        .greediness = s.greedyness,
                        .max_iterations = s.max_iterations,
                        .traversability_threshold = s.traversability_threshold,
                        .grow_to_point_neighbors = s.grow_to_point_neighbors,
                        .opt_metric = s.opt_metric,
                        .threshold_step = s.threshold_step
                };
                RRTStarPlanner planner( &map, params );
                int res = planner.build_plan( p.start_position, p.start_yaw,
                                              p.target_position, p.target_yaw );
                if( res == -1 )
                    std::cout << RED << "\t\tTarget not reached" << RESET << std::endl;
                else
                    std::cout << GREEN << "\t\tTarget reached" << RESET << std::endl;
                string out_file = build_filename( p, s );
                std::cout << "\t\tResult saved in " << out_file << std::endl;
                write_svg_of_map_and_plan( out_file, map, planner );
            }
        }

        if( argc > 1 && string(argv[1]) == "--first" )
            return 0;
    }

    return 0;
}
