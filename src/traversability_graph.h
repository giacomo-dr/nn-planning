// traversability_graph.h
// Author: Giacomo Del Rio
// Date: 4 September 2017

#ifndef TRAVERSABILITY_GRAPH_H
#define TRAVERSABILITY_GRAPH_H

#include <memory>
#include <string>
#include "antlr4-runtime.h"
#include "DOTBaseVisitor.h"


/*
  The array probabilities holds the traversability information
  in this arrangement, for each (x,y) there are 8 directions,
  indexed from 0 to 7, for which the probability is given:

                      [2] Pi/4
                [3] *    |    * [1]
                      *  |  *
                        *|*
           Pi [4]  ------ ------  [0] 0=2Pi
                        *|*
                      *  |  *
                [5] *    |    * [7]
                      [6] 3/2Pi
 */
class TraversabilityGraph {
public:
    TraversabilityGraph( int n_rows, int n_columns );
    virtual ~TraversabilityGraph(){};

    void load_from_dotfile( std::string filename );
    double& get( int x, int y, int dir );
    double getLinear( double x, double y, double alpha );

    int nrows();
    int ncolumns();

private:
    int n_rows, n_columns;       // Number of rows and columns of the graph
    std::unique_ptr<double[]> probabilities;  // Probabilities: size = n_rows * n_columns * 8
};


class TGraphDotVisitor : public nnplanning::DOTBaseVisitor {
public:
    TGraphDotVisitor( TraversabilityGraph& tg ) : tg(tg) {};
    virtual antlrcpp::Any visitEdge_stmt( nnplanning::DOTParser::Edge_stmtContext *ctx ) override;

private:
    TraversabilityGraph& tg;
};

#endif //TRAVERSABILITY_GRAPH_H
