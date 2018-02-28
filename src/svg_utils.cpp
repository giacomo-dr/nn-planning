// svg_utils.cpp
// Author: Giacomo Del Rio
// Date: 23 Apr 2017

#include <string>
#include "svg_utils.h"

namespace svg { namespace utils {

double scale_factor = 10.0;
double map_padding = 1.0;

void initialize_svg_writer( SVGWriter& svg, const HeightMap &map ){
    double min_x = -(map.size_x_mt() / 2.0 + map_padding) * scale_factor;
    double min_y = -(map.size_y_mt() / 2.0 + map_padding) * scale_factor;
    double width = (map.size_x_mt() + map_padding * 2.0) * scale_factor;
    double height = (map.size_y_mt() + map_padding * 2.0) * scale_factor;
    svg.set_global_transform( "matrix(1 0 0 -1 0 0)" );
    using std::to_string;
    svg.set_viewbox( to_string(min_x) + " " + to_string(min_y) + " " +
                     to_string(width) + " " + to_string(height) );
}

void write_height_map( SVGWriter& svg, const HeightMap& map ){
    double min_x = -(map.size_x_mt() / 2.0) * scale_factor;
    double min_y = -(map.size_y_mt() / 2.0) * scale_factor;
    double width = map.size_x_mt() * scale_factor;
    double height = map.size_y_mt() * scale_factor;
    svg.write_image( min_x, min_y, width, height,
                     "image/png", map.encode_base64(".png") );
}

void write_rrt_plan( SVGWriter& svg, const RRTPlan& plan ){
    // Find the node with min value of probability
    auto min_res = std::min_element( plan.nodes.begin(), plan.nodes.end(),
                                     [](const RRTNode& a, const RRTNode& b) { return a.probability < b.probability; } );
    double min_prob = (*min_res).probability * 1.25;
    double norm_factor = -1.0 / min_prob;

    // Print nodes and edges
    for( const RRTNode& p: plan.nodes ){
        double intensity = (p.probability - min_prob) * norm_factor;
        std::ostringstream rgba_str;
        rgba_str << "rgba(255,0,0," << intensity << ")";
        const char* rgba = rgba_str.str().c_str();
        svg.set_style_property( "fill", rgba );
        svg.set_style_property( "stroke", rgba );
        svg.set_style_property( "stroke-width", "0" );

//        svg.write_circle( p.vertex.x() * scale_factor,
//                          p.vertex.y() * scale_factor, 0.3 );
        if( p.parent != -1 ) {
            svg.set_style_property( "stroke-width", "0.1" );
            svg.write_segment(p.vertex.x() * scale_factor,
                              p.vertex.y() * scale_factor,
                              plan.nodes[p.parent].vertex.x() * scale_factor,
                              plan.nodes[p.parent].vertex.y() * scale_factor);
        }
    }
}

void write_rrt_star_plan( SVGWriter& svg, const RRTStarPlanner& planner ){
    // Find the node with min value of probability
    const RRTStarPlan& plan = planner.get_plan();
    auto min_res = std::min_element( plan.nodes.begin(), plan.nodes.end(),
                                     [](const RRTStarNode& a, const RRTStarNode& b) { return a.probability < b.probability; } );
    double min_prob = (*min_res).probability * 1.25;
    double norm_factor = -1.0 / min_prob;

    // Print start/end point
    svg.set_style_property( "fill", "rgba(255,255,0,1)" );
    svg.set_style_property( "stroke", "rgba(255,255,0,1)" );
    svg.set_style_property( "stroke-width", "0.3" );
    svg.write_circle( planner.get_start_point().x() * scale_factor,
                      planner.get_start_point().y() * scale_factor, 0.6 );
    svg.set_style_property( "fill", "rgba(0,0,0,1)" );
    svg.write_circle( planner.get_target_point().x() * scale_factor,
                      planner.get_target_point().y() * scale_factor, 0.8 );

    // Print nodes and edges
    for( const RRTStarNode& p: plan.nodes ){
        double intensity = (p.probability - min_prob) * norm_factor;
        std::ostringstream rgba_str;
        rgba_str << "rgba(0,255,0," << intensity << ")";
        const char* rgba = rgba_str.str().c_str();
        svg.set_style_property( "fill", rgba );
        svg.set_style_property( "stroke", rgba );
        svg.set_style_property( "stroke-width", "0" );

//        svg.write_circle( p.vertex.x() * scale_factor,
//                          p.vertex.y() * scale_factor, 0.3 );
        if( p.parent != -1 ) {
            svg.set_style_property( "stroke-width", "0.1" );
            svg.write_segment(p.vertex.x() * scale_factor,
                              p.vertex.y() * scale_factor,
                              plan.nodes[p.parent].vertex.x() * scale_factor,
                              plan.nodes[p.parent].vertex.y() * scale_factor);
        }
    }
}

void write_path( SVGWriter& svg, const WaypointPath2D& path ){
    if( path.waypoints.size() < 2 )
        return;

    svg.set_style_property( "fill", "rgba(255,255,0,1)" );
    svg.set_style_property( "stroke", "rgba(255,255,0,1)" );
    svg.set_style_property( "stroke-width", "0.3" );

    const auto& pts = path.waypoints;
    svg.write_circle( pts.front().x() * scale_factor,
                      pts.front().y() * scale_factor, 0.6 );
    for( int i = 1 ; i < path.waypoints.size() ; i++ ){
        svg.write_segment( pts[i-1].x() * scale_factor,
                           pts[i-1].y() * scale_factor,
                           pts[i].x() * scale_factor,
                           pts[i].y() * scale_factor);
    }
    svg.set_style_property( "fill", "rgba(0,0,0,1)" );
    svg.write_circle( pts.back().x() * scale_factor,
                      pts.back().y() * scale_factor, 0.8 );
}

}} //namespace svg:utils