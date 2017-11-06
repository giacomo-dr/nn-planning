// antlr-main.cpp
// Author: Giacomo Del Rio
// Date: 1 Sep 2017


#include <iostream>
#include <iomanip>
#include <math.h>
#include "lodepng.h"
#include "traversability_graph.h"
#include "height_map.h"

#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"

void encodeOneStep( const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height )
{
    //Encode the image
    unsigned error = lodepng::encode( filename, image, width, height );

    //if there's an error, display it
    if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}

int main() {

    TraversabilityGraph tg( 64, 64 );
    tg.load_from_dotfile( TG_FILENAME );
    HeightMap map( MAP_FILENAME, 10, 0.4 );
    map.load_traversability_graph( TG_FILENAME, 64, 64 );

//    std::cout << tg.get( 1, 1, 0 ) << std::endl;
//    std::cout << tg.get( 1, 1, 1 ) << std::endl;
//    std::cout << tg.get( 1, 1, 2 ) << std::endl;
//    std::cout << tg.get( 1, 1, 3 ) << std::endl;
//    std::cout << tg.get( 1, 1, 4 ) << std::endl;
//    std::cout << tg.get( 1, 1, 5 ) << std::endl;
//    std::cout << tg.get( 1, 1, 6 ) << std::endl;
//    std::cout << tg.get( 1, 1, 7 ) << std::endl;

    std::cout << tg.get( 63/2 + 1, 63/2 + 1, 0 ) << std::endl;
    //std::cout << map.traversability_prob( MapOrigin::CENTER_CENTER, 10.0/63.0, 10.0/63.0, 0 ) << std::endl;
    std::cout << map.traversability_prob( MapOrigin::CENTER_CENTER, 10.0/63.0 + 10.0/126.0,
                                          10.0/63.0 + 10.0/126.0, 0 ) << std::endl;

//    std::cout << tg.get( 1, 1, 2 ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, 3.0/2.0*M_PI ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, M_PI / 4.0 ) << std::endl;
//    std::cout << tg.getLinear( 1.0, 1.0, M_PI / 8.0 ) << std::endl;

    // Generate an series of images of tg with various angles
    unsigned width = 64*10, height = 64*10;
    std::vector<unsigned char> image;
    image.resize(width * height * 4);
    int i = 0;
    for( double angle = 0 ; angle < M_PI * 2.0 ; i++, angle += 0.01 ){
        for (unsigned y = 0; y < height; y++)
            for (unsigned x = 0; x < width; x++) {
                double xd = (x / 640.0) * 63.0;
                double yd = (y / 640.0) * 63.0;

                //std::cout << xd << "," << yd << std::endl;
                unsigned char val = tg.getLinear( xd, yd, angle ) * 255;
                image[4 * width * y + 4 * x + 0] = val;
                image[4 * width * y + 4 * x + 1] = val;
                image[4 * width * y + 4 * x + 2] = val;
                image[4 * width * y + 4 * x + 3] = 255;
            }
        std::ostringstream fname;
        fname << "video/tgraph_" << std::setfill('0') << std::setw(4) << i << ".png";
        encodeOneStep( fname.str().c_str(), image, width, height );
        std::cout << angle << std::endl;
    }

    return 0;
}

// $ dot -Kfdp -n -Tpng -o sample.png ../../traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot