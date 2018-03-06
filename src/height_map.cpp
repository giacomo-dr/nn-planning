// height_map.cpp
// Author: Giacomo Del Rio
// Date: 21 Apr 2017

#include <vector>
#include "height_map.h"
#include "base64.h"


HeightMap::HeightMap(){
}

HeightMap::HeightMap( std::string filename, double x_meters, double z_meters ){
    if( !load_image(filename) )
        throw std::runtime_error( "Can't load file '" + filename + "'" );
    set_dimensions( x_meters, z_meters );
}

bool HeightMap::load_image( std::string filename ){
    image = cv::imread( filename, cv::IMREAD_GRAYSCALE );
    return image.data != nullptr;
}

void HeightMap::load_traversability_graph( std::string filename, int n_rows, int n_columns,
                                           double padding_x, double padding_y ){
    t_graph.reset( new TraversabilityGraph( n_rows, n_columns ) );
    t_graph->load_from_dotfile( filename );
    this->padding_x = padding_x;
    this->padding_y = padding_y;
}

void HeightMap::set_dimensions( double x_meters, double z_meters ){
    this->x_meters = x_meters;
    this->y_meters = x_meters * (double)image.rows / (double)image.cols;
    this->z_meters = z_meters;
    uniform_x = std::uniform_real_distribution<double>( 0, x_meters );
    uniform_y = std::uniform_real_distribution<double>( 0, y_meters );
}

int HeightMap::size_x_px() const {
    return image.cols;
}

int HeightMap::size_y_px() const {
    return image.rows;
}

double HeightMap::size_x_mt() const {
    return x_meters;
}

double HeightMap::size_y_mt() const {
    return y_meters;
}

double HeightMap::size_z_mt() const {
    return z_meters;
}

const unsigned char* HeightMap::get_data() const {
    return image.ptr();
}

std::string HeightMap::encode_base64( std::string format ) const {
    cv::Mat image_flipped;
    cv::flip( image, image_flipped, 0 );
    std::vector<unsigned char> buf;
    cv::imencode( format, image_flipped, buf );
    return base64_encode( reinterpret_cast<const unsigned char*>( &buf[0] ),
                          buf.size() );
}

//std::vector<Point2D> randoms = { Point2D(1, -3), Point2D(1, -2), Point2D(3, -1) };
Point2D HeightMap::sample_Cfree( MapOrigin o ){
//    Point2D p = randoms.at( 0 );
//    randoms.erase( randoms.begin() );
//    return p;
    if( o == MapOrigin::CENTER_CENTER )
        return Point2D( uniform_x(rng) - x_meters / 2.0,
                        uniform_y(rng) - y_meters / 2.0 );
    else
        return Point2D( uniform_x(rng), uniform_y(rng) );
}

void HeightMap::seed_Cfree_sampler(){
    rng.seed();
}

double HeightMap::traversability_prob( MapOrigin o, double pos_x, double pos_y, double yaw ) {
    assert(0 <= yaw && yaw < M_PI * 2.0);

    if( o == MapOrigin::CENTER_CENTER ){
        assert(-x_meters / 2.0 <= pos_x && pos_x < x_meters / 2.0);
        assert(-y_meters / 2.0 <= pos_y && pos_y < y_meters / 2.0);
        if( pos_x < -x_meters / 2.0 + padding_x || pos_x >= x_meters / 2.0 - padding_x ||
                pos_y < -y_meters / 2.0 + padding_y || pos_y >= y_meters / 2.0 - padding_y )
            return 0;
        return t_graph->getLinear((0.5 + pos_x / (x_meters - 2.0 * padding_x)) * (double)(t_graph->ncolumns()-1),
                                  (0.5 - pos_y / (y_meters - 2.0 * padding_y)) * (double)(t_graph->nrows()-1),
                                  yaw);
    }else{
        assert( 0 <= pos_x && pos_x < x_meters );
        assert( 0 <= pos_y && pos_y < y_meters );
        if( pos_x < padding_x || pos_x >= x_meters - padding_x ||
                pos_y < padding_y || pos_y >= y_meters - padding_y )
            return 0;
        return t_graph->getLinear(((pos_x - padding_x) / (x_meters - 2.0 * padding_x)) * (double)(t_graph->ncolumns()-1),
                                  ((pos_y - padding_y) / (y_meters - 2.0 * padding_y)) * (double)(t_graph->nrows()-1),
                                  yaw);
    }
}

cv::Mat HeightMap::extract_patch( double center_x, double center_y,
                                  double width, double height,
                                  double angle, MapOrigin o ){
    cv::Point2f patch_center;
    if( o == MapOrigin::CENTER_CENTER )
        patch_center = cv::Point2f( (center_x + x_meters / 2.0) / x_meters * image.cols,
                                    image.rows - (center_y + y_meters / 2.0) / y_meters * image.rows );
    else
        patch_center = cv::Point2f( center_x / x_meters * image.cols,
                                    image.rows - center_y / y_meters * image.rows );
    cv::Point2f patch_size( width / x_meters * image.cols,
                            height / y_meters * image.rows );
//    std::cout << "    Image patch center (" << patch_center.x << ", " << patch_center.y << ")\n";
//    std::cout << "    Image patch size (" << patch_size.x << ", " << patch_size.y << ")\n";
//    std::cout << "    Image patch angle (" << 90.0 - angle * 180.0 / M_PI << ")\n";
    cv::Mat image_rotated, patch;
    cv::RotatedRect rect( patch_center, patch_size, 90.0 - angle * 180.0 / M_PI );
    cv::Mat M = getRotationMatrix2D( rect.center, rect.angle, 1.0 );
    warpAffine( image, image_rotated, M, image.size(), cv::INTER_CUBIC );
    getRectSubPix( image_rotated, rect.size, rect.center, patch );

//    namedWindow( "Display Image", cv::WINDOW_AUTOSIZE );
//    imshow( "Display Image", patch );
//    cv::waitKey( 0 );

    return patch;
}