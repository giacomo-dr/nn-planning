// svg_utils.h
// Author: Giacomo Del Rio
// Date: 23 Apr 2017

#ifndef SVG_UTILS_H
#define SVG_UTILS_H

#import "svg_writer.h"
#import "rrt_star_planner.h"
#import "height_map.h"

namespace svg { namespace utils {

extern double scale_factor;
extern double map_padding;

void initialize_svg_writer( SVGWriter& svg, const HeightMap &map );

void write_height_map( SVGWriter& svg, const HeightMap &map );

void write_rrt_star_plan( SVGWriter& svg, const RRTStarPlan &plan );

void write_path( SVGWriter& svg, const WaypointPath2D& path );

}} //namespace svg::utils

#endif //SVG_UTILS_H
