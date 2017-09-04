// antlr-main.cpp
// Author: Giacomo Del Rio
// Date: 1 Sep 2017

#include <iostream>
#include <fstream>
#include "antlr4-runtime.h"
#include "DOTLexer.h"
#include "DOTParser.h"
#include "DOTBaseListener.h"
#include "DOTBaseVisitor.h"


//class DotListener : public nnplanning::DOTBaseListener {
//public:
//    void enterGraph( nnplanning::DOTParser::GraphContext *ctx ) override {
//        std::cout << "Magnesio\n";
//    }
//};

class DotVisitor : public nnplanning::DOTBaseVisitor {
public:
    virtual antlrcpp::Any visitGraph( nnplanning::DOTParser::GraphContext *ctx ) override {
        std::cout << "Magnesio\n";
        return visitChildren(ctx);
    }
};


int main() {
    std::ifstream stream;
    stream.open( "../../traversability_graphs_dataset/graphs/t_graph_cnn_heightmap1_full.dot" );
    antlr4::ANTLRInputStream input( stream );
    nnplanning::DOTLexer lexer( &input );
    antlr4::CommonTokenStream tokens( &lexer );
    nnplanning::DOTParser parser( &tokens );
    antlr4::tree::ParseTree *tree = parser.graph();

    DotVisitor visitor;
    visitor.visit( tree );
//    DotListener listener;
//    antlr4::tree::ParseTreeWalker::DEFAULT.walk( &listener, tree );

    return 0;
}

// $ dot -Kfdp -n -Tpng -o sample.png ../../traversability_graphs_dataset/graphs/t_graph_cnn_custom9_full.dot
