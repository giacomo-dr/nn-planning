// antlr-main.cpp
// Author: Giacomo Del Rio
// Date: 1 Sep 2017


#include <iostream>
#include <iomanip>
#include <cmath>
#include "lodepng.h"
#include "reach_target_task.h"

#define CUSTOM

#ifdef CUSTOM
    #define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
    #define MAP_X_METERS 10.0
    #define MAP_HEIGHT 0.4
    #define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"
    #define TG_SIZE 64
#endif

#ifdef ROCKS
    #define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/arc_rocks.png"
    #define MAP_X_METERS 10.0
    #define MAP_HEIGHT 0.4
    #define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_arc_rocks_full.dot"
    #define TG_SIZE 64
#endif

void saveImage(const char *filename, std::vector<unsigned char> &image, unsigned width, unsigned height)
{
    unsigned error = lodepng::encode( filename, image, width, height );
    if( error )
        std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}

void generateThresholdMap( HeightMap& map, double treshold, Point2D step, std::string filename ) {
    // Generate an image with a blue pixel where the step is traversable, with a given threshold
    unsigned img_width = 64*10, img_height = 64*10;
    std::vector<unsigned char> image;
    image.resize(img_width * img_height * 4); // 8-bit RGBA
    RRTPlanner planner( &map, 0.3, 100, 5000, M_PI_4, treshold );
    for( unsigned y = img_height; y > 0 ; y-- )
        for( unsigned x = 0; x < img_width; x++ ){
            double xd = (x / (double)img_width) * map.size_x_mt() - map.size_x_mt() / 2.0;
            double yd = (1 - (y / (double)img_height)) * map.size_y_mt() - map.size_y_mt() / 2.0;

//            std::cout << "y = " << y << " size_y_mt = " << map.size_y_mt() << std::endl;
//            std::cout << "xd = " << xd << " yd = " << yd << std::endl;
            unsigned char val = static_cast<unsigned char>(
                    planner.is_traversable(Point2D(xd, yd), Point2D(xd, yd) + step) ? 255 : 0 );
            image[4 * img_width * y + 4 * x + 0] = 0;
            image[4 * img_width * y + 4 * x + 1] = 0;
            image[4 * img_width * y + 4 * x + 2] = val;
            image[4 * img_width * y + 4 * x + 3] = val;
        }
    saveImage( filename.c_str(), image, img_width, img_height );
}

int main() {

    TraversabilityGraph tg( TG_SIZE, TG_SIZE );
    tg.load_from_dotfile( TG_FILENAME );
    HeightMap map( MAP_FILENAME, MAP_X_METERS, MAP_HEIGHT );
    map.load_traversability_graph( TG_FILENAME, TG_SIZE, TG_SIZE );

    generateThresholdMap( map, 0.8, Point2D(0.1, 0.0), "tgraphs/tgraph_tv_right.png" );
    generateThresholdMap( map, 0.8, Point2D(-0.1, 0.0), "tgraphs/tgraph_tv_left.png" );
    generateThresholdMap( map, 0.8, Point2D(0.0, 0.1), "tgraphs/tgraph_tv_up.png" );
    generateThresholdMap( map, 0.8, Point2D(0.0, -0.1), "tgraphs/tgraph_tv_down.png" );

//    std::cout << tg.get( 1, 1, 0 ) << std::endl;
//    std::cout << tg.get( 1, 1, 1 ) << std::endl;
//    std::cout << tg.get( 1, 1, 2 ) << std::endl;
//    std::cout << tg.get( 1, 1, 3 ) << std::endl;
//    std::cout << tg.get( 1, 1, 4 ) << std::endl;
//    std::cout << tg.get( 1, 1, 5 ) << std::endl;
//    std::cout << tg.get( 1, 1, 6 ) << std::endl;
//    std::cout << tg.get( 1, 1, 7 ) << std::endl;

    //std::cout << tg.get( 63/2 + 1, 63/2 + 1, 0 ) << std::endl;
    //std::cout << map.traversability_prob( MapOrigin::CENTER_CENTER, 10.0/63.0, 10.0/63.0, 0 ) << std::endl;
    //std::cout << map.traversability_prob( MapOrigin::CENTER_CENTER, 10.0/63.0 + 10.0/126.0,
    //                                      10.0/63.0 + 10.0/126.0, 0 ) << std::endl;

//    std::cout << tg.get( 1, 1, 2 ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, 3.0/2.0*M_PI ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, M_PI / 4.0 ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, M_PI / 8.0 ) << std::endl;

    // Generate an series of images of tg with various angles
//    unsigned width = 64*10, height = 64*10;
//    std::vector<unsigned char> image;
//    image.resize(width * height * 4);
//    int i = 0;
//    for( double angle = 0 ; angle < M_PI * 2.0 ; i++, angle += M_PI / 2.0 ){
//        for (unsigned y = 0; y < height; y++)
//            for (unsigned x = 0; x < width; x++) {
//                double xd = (x / 640.0) * 63.0;
//                double yd = (y / 640.0) * 63.0;
//
//                //std::cout << xd << "," << yd << std::endl;
//                unsigned char val = (unsigned char)(tg.getLinear( xd, yd, angle ) * 255);
//                image[4 * width * y + 4 * x + 0] = val;
//                image[4 * width * y + 4 * x + 1] = val;
//                image[4 * width * y + 4 * x + 2] = val;
//                image[4 * width * y + 4 * x + 3] = 255;
//            }
//        std::ostringstream fname;
//        fname << "tgraphs/tgraph_" << std::setfill('0') << std::setw(4) << i << ".png";
//        std::cout << fname.str() << std::endl;
//        encodeOneStep( fname.str().c_str(), image, width, height );
//        std::cout << angle << std::endl;
//    }
//
//    // Do the same as before but querying map instead of tg
//    image.resize(width * height * 4);
//    i = 0;
//    for( double angle = 0 ; angle < M_PI * 2.0 ; i++, angle += M_PI / 2.0 ){
//        for (unsigned y = 0; y < height; y++)
//            for (unsigned x = 0; x < width; x++) {
//                double xd = (x / 640.0) * map.size_x_mt();
//                double yd = (y / 640.0) * map.size_y_mt();
//
//                //std::cout << xd << "," << yd << std::endl;
//                unsigned char val = (unsigned char)
//                        (map.traversability_prob( MapOrigin::TOP_LEFT, xd, yd, angle ) * 255);
//                image[4 * width * y + 4 * x + 0] = val;
//                image[4 * width * y + 4 * x + 1] = val;
//                image[4 * width * y + 4 * x + 2] = val;
//                image[4 * width * y + 4 * x + 3] = 255;
//            }
//        std::ostringstream fname;
//        fname << "tgraphs/tgraph_" << std::setfill('0') << std::setw(4) << i << "_map.png";
//        std::cout << fname.str() << std::endl;
//        encodeOneStep( fname.str().c_str(), image, width, height );
//        std::cout << angle << std::endl;
//    }

    return 0;
}

// $ dot -Kfdp -n -Tpng -o sample.png ../../traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot