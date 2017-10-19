// height_map.h
// Author: Giacomo Del Rio
// Date: 21 Apr 2017

#ifndef HEIGHT_MAP_H
#define HEIGHT_MAP_H

#include <string>
#include <random>
#include <opencv2/opencv.hpp>
#include "geometry.h"
#include "traversability_graph.h"


enum class MapOrigin { CENTER_CENTER, TOP_LEFT };


class HeightMap {
public:
    HeightMap();
    HeightMap( std::string filename, double x_meters, double z_meters );
    virtual ~HeightMap(){}

    bool load_image( std::string filename );
    void load_traversability_graph( std::string filename, int n_rows, int n_columns );
    void set_dimensions( double x_meters, double z_meters );
    int size_x_px() const;
    int size_y_px() const;
    double size_x_mt() const;
    double size_y_mt() const;
    double size_z_mt() const;
    const unsigned char* get_data() const;
    std::string encode_base64( std::string format ) const;
    cv::Mat extract_patch( double center_x, double center_y,
                           double width, double height,
                           double angle, MapOrigin o );
    Point2D sample_Cfree( MapOrigin o );
    double traversability_prob( MapOrigin o, double pos_x, double pos_y, double yaw );

private:
    cv::Mat image;
    double x_meters;
    double y_meters;
    double z_meters;
    std::unique_ptr<TraversabilityGraph> t_graph;

    std::mt19937 rng;
    std::uniform_real_distribution<double> uniform_x;
    std::uniform_real_distribution<double> uniform_y;
};

#endif //HEIGHT_MAP_H
