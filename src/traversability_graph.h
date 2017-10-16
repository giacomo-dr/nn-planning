// traversability_graph.h
// Author: Giacomo Del Rio
// Date: 4 September 2017

#ifndef TRAVERSABILITY_GRAPH_H
#define TRAVERSABILITY_GRAPH_H

#include <memory>
#include <string>
#include "antlr4-runtime.h"
#include "DOTBaseVisitor.h"


class TraversabilityGraph {
public:
    TraversabilityGraph( int n_rows, int n_columns, double step_meters );
    virtual ~TraversabilityGraph(){};

    void load_from_dotfile( std::string filename );
    float& get( int x, int y, int dir );
    int nrows();
    int ncolumns();
    double step();

private:
    int n_rows, n_columns;       // Number of rows and columns of the graph
    double step_meters;          // Distance, in meters, between two adjacent rows or columns
    std::unique_ptr<float[]> probabilities;  // Probabilities: size = n_rows * n_columns * 8
};


class TGraphDotVisitor : public nnplanning::DOTBaseVisitor {
public:
    TGraphDotVisitor( TraversabilityGraph& tg ) : tg(tg) {};
    virtual antlrcpp::Any visitEdge_stmt( nnplanning::DOTParser::Edge_stmtContext *ctx ) override;

private:
    TraversabilityGraph& tg;
};

#endif //TRAVERSABILITY_GRAPH_H
