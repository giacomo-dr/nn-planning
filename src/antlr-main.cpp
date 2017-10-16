// antlr-main.cpp
// Author: Giacomo Del Rio
// Date: 1 Sep 2017


#include <iostream>
#include "traversability_graph.h"


int main() {

    TraversabilityGraph tg( 64, 64, 1 );
    tg.load_from_dotfile( "/Users/delrig/Downloads/Thesis/traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot" );

    std::cout << tg.get( 1, 1, 0 ) << std::endl;
    std::cout << tg.get( 1, 1, 1 ) << std::endl;
    std::cout << tg.get( 1, 1, 2 ) << std::endl;
    std::cout << tg.get( 1, 1, 3 ) << std::endl;
    std::cout << tg.get( 1, 1, 4 ) << std::endl;
    std::cout << tg.get( 1, 1, 5 ) << std::endl;
    std::cout << tg.get( 1, 1, 6 ) << std::endl;
    std::cout << tg.get( 1, 1, 7 ) << std::endl;
    return 0;
}

// $ dot -Kfdp -n -Tpng -o sample.png ../../traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot
