// antlr-main.cpp
// Author: Giacomo Del Rio
// Date: 1 Sep 2017


#include <iostream>
#include <math.h>
#include "traversability_graph.h"
#include "height_map.h"

#define MAP_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/heightmaps/custom9.png"
#define TG_FILENAME "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot"

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

    return 0;
}

// $ dot -Kfdp -n -Tpng -o sample.png ../../traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot