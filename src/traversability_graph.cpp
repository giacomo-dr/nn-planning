// traversability_graph.cpp
// Author: Giacomo Del Rio
// Date: 4 September 2017

#include <cassert>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "traversability_graph.h"
#include "DOTLexer.h"
#include "DOTParser.h"

using namespace nnplanning;


antlrcpp::Any TGraphDotVisitor::visitEdge_stmt( DOTParser::Edge_stmtContext *ctx ){
    // Extract source, destination and probability of traversing
    int src = std::stoi( ctx->node_id()->getText() );
    int dst = std::stoi( ctx->edgeRHS()->node_id( 0 )->getText() );
    float prob = 0.0;
    for( int i = 0 ; i < ctx->attr_list()->a_list(0)->id().size() ; i++ )
        if (ctx->attr_list()->a_list(0)->id(i)->getText() == "probability") {
            prob = (float)atof( ctx->attr_list()->a_list(0)->id(i+1)->getText().c_str() );
            break;
        }

    // Put the probability information into the TraversabilityGraph
    int src_x = src % tg.ncolumns();
    int src_y = src / tg.ncolumns();
    if( dst == src + 1 ){
        tg.get( src_x, src_y, 2 ) = prob; // E
    }else if( dst == src - 1 ){
        tg.get( src_x, src_y, 6 ) = prob; // W
    }else if( dst == src + tg.ncolumns() ){
        tg.get( src_x, src_y, 4 ) = prob; // S
    }else if( dst == src - tg.ncolumns() ){
        tg.get( src_x, src_y, 0 ) = prob; // N
    }else if( dst == src + tg.ncolumns() + 1 ){
        tg.get( src_x, src_y, 3 ) = prob; // SE
    }else if( dst == src + tg.ncolumns() - 1 ){
        tg.get( src_x, src_y, 5 ) = prob; // SW
    }else if( dst == src - tg.ncolumns() + 1 ){
        tg.get( src_x, src_y, 1 ) = prob; // NE
    }else if( dst == src - tg.ncolumns() - 1 ){
        tg.get( src_x, src_y, 7 ) = prob; // NW
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
    probabilities.reset( new float[n_rows * n_columns * 8] );
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

float& TraversabilityGraph::get( int x, int y, int dir ){
    assert( 0 <= x && x < n_columns);
    assert( 0 <= y && y < n_rows);

    // y * n_columns * 8 + x * 8 + dirs = (y * n_columns + x) * 8 + dirs
    //std::cout << "Get: " << (y * n_columns + x) * 8 + dir << std::endl;
    return probabilities[(y * n_columns + x) * 8 + dir];
}

int TraversabilityGraph::nrows(){ return n_rows; }

int TraversabilityGraph::ncolumns(){ return n_columns; }

double TraversabilityGraph::step(){ return step_meters; }