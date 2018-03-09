// svg_utils.h
// Author: Giacomo Del Rio
// Date: 23 Apr 2017

#ifndef SVG_UTILS_H
#define SVG_UTILS_H

#include "svg_writer.h"
#include "rrt_planner.h"
#include "rrt_star_planner.h"
#include "height_map.h"

namespace svg { namespace utils {

extern double scale_factor;
extern double map_padding;

void initialize_svg_writer( SVGWriter& svg, const HeightMap &map );

void write_height_map( SVGWriter& svg, const HeightMap &map );

void write_rrt_plan( SVGWriter& svg, const RRTPlanner &plan );
void write_rrt_star_plan( SVGWriter& svg, const RRTStarPlanner &plan );

void write_path( SVGWriter& svg, const WaypointPath2D& path );

}} //namespace svg::utils

#endif //SVG_UTILS_H
