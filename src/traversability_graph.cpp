// traversability_graph.cpp
// Author: Giacomo Del Rio
// Date: 4 September 2017

#include <cassert>
#include <iostream>
#include <fstream>
#include <math.h>
#include "traversability_graph.h"
#include "DOTLexer.h"
#include "DOTParser.h"

using namespace nnplanning;


antlrcpp::Any TGraphDotVisitor::visitEdge_stmt( DOTParser::Edge_stmtContext *ctx ){
    // Extract source, destination and probability of traversing
    int src = std::stoi( ctx->node_id()->getText() );
    int dst = std::stoi( ctx->edgeRHS()->node_id( 0 )->getText() );
    double prob = 0.0;
    for( int i = 0 ; i < ctx->attr_list()->a_list(0)->id().size() ; i++ )
        if (ctx->attr_list()->a_list(0)->id(i)->getText() == "probability") {
            prob = atof( ctx->attr_list()->a_list(0)->id(i+1)->getText().c_str() );
            break;
        }

    // Put the probability information into the TraversabilityGraph
    int src_x = src % tg.ncolumns();
    int src_y = src / tg.ncolumns();
    if( dst == src + 1 ){
        tg.get( src_x, src_y, 0 ) = prob; // E
    }else if( dst == src - 1 ){
        tg.get( src_x, src_y, 4 ) = prob; // W
    }else if( dst == src + tg.ncolumns() ){
        tg.get( src_x, src_y, 6 ) = prob; // S
    }else if( dst == src - tg.ncolumns() ){
        tg.get( src_x, src_y, 2 ) = prob; // N
    }else if( dst == src + tg.ncolumns() + 1 ){
        tg.get( src_x, src_y, 7 ) = prob; // SE
    }else if( dst == src + tg.ncolumns() - 1 ){
        tg.get( src_x, src_y, 5 ) = prob; // SW
    }else if( dst == src - tg.ncolumns() + 1 ){
        tg.get( src_x, src_y, 1 ) = prob; // NE
    }else if( dst == src - tg.ncolumns() - 1 ){
        tg.get( src_x, src_y, 3 ) = prob; // NW
    }else{
        abort();
    }

    std::cout << src << " -> " << dst << " [" << prob << "]" << std::endl;
    return visitChildren(ctx);
}

TraversabilityGraph::TraversabilityGraph( int n_rows, int n_columns,
                                          double step_meters ){
    this->n_rows = n_rows;
    this->n_columns = n_columns;
    this->step_meters = step_meters;
    probabilities.reset( new double[n_rows * n_columns * 8] );
    for( int i = 0 ; i < n_rows * n_columns * 8 ; ++i )
        probabilities[i] = 0.0;
}

void TraversabilityGraph::load_from_dotfile( std::string filename ){
    std::ifstream stream;
    stream.open( filename );
    antlr4::ANTLRInputStream input( stream );
    nnplanning::DOTLexer lexer( &input );
    antlr4::CommonTokenStream tokens( &lexer );
    nnplanning::DOTParser parser( &tokens );
    antlr4::tree::ParseTree *tree = parser.graph();

    TGraphDotVisitor visitor( *this );
    visitor.visit( tree );
}

double& TraversabilityGraph::get( int x, int y, int dir ){
    assert( 0 <= x && x < n_columns);
    assert( 0 <= y && y < n_rows);

    // y * n_columns * 8 + x * 8 + dirs = (y * n_columns + x) * 8 + dirs
    //std::cout << "Get: " << (y * n_columns + x) * 8 + dir << std::endl;
    return probabilities[(y * n_columns + x) * 8 + dir];
}

double TraversabilityGraph::getLinear( double x, double y, double alpha ) {
    assert( 0 <= x && x < n_columns - 1 );
    assert( 0 <= y && y < n_rows - 1 );
    assert( 0 <= alpha && alpha < M_PI * 2.0 );

    // Trilinear interpolation: en.wikipedia.org/wiki/Trilinear_interpolation
    double x0 = floor( x ), x1 = x0 + 1;
    double y0 = floor( y ), y1 = y0 + 1;
    double xd = x - x0;
    double yd = y - y0;
    double a = alpha / (2.0 * M_PI) * 8.0; // [0, 2Pi] -> [0, 8]
    double a0 = floor( a ), a1 = ((int)a0 + 1) % 8;
    double ad = a - a0;

    std::cout << "x0: " << x0 << "  x1: " << x1 << std::endl;
    std::cout << "y0: " << y0 << "  y1: " << y1 << std::endl;
    std::cout << "a0: " << a0 << "  a1: " << a1 << std::endl;

    double c00 = get(x0, y0, a0) * (1.0 - xd) + get(x1, y0, a0) * xd;
    double c01 = get(x0, y0, a1) * (1.0 - xd) + get(x1, y0, a1) * xd;
    double c10 = get(x0, y1, a0) * (1.0 - xd) + get(x1, y1, a0) * xd;
    double c11 = get(x0, y1, a1) * (1.0 - xd) + get(x1, y1, a1) * xd;

    double c0 = c00 * (1.0 - yd) + c10 * yd;
    double c1 = c01 * (1.0 - yd) + c11 * yd;

    return c0 * (1.0 - ad) + c1 * ad;
}

int TraversabilityGraph::nrows(){ return n_rows; }

int TraversabilityGraph::ncolumns(){ return n_columns; }

double TraversabilityGraph::step(){ return step_meters; }