// svg_writer.cpp
// Author: Giacomo Del Rio
// Date: 20 Apr 2017

#include "svg_writer.h"

namespace svg {

SVGWriter::SVGWriter( std::string filename ){
    this->filename = filename;
}

SVGWriter::~SVGWriter(){
    if( svg.is_open() )
        svg.close();
}

void SVGWriter::begin(){
    svg.open( filename.c_str() );
    write_prologue();
}

void SVGWriter::end(){
    write_epilogue();
    svg.close();
}

void SVGWriter::set_style_property( std::string name, std::string value ){
    properties[name] = value;
}

void SVGWriter::clear_style_property( std::string name ){
    properties.erase( name );
}

void SVGWriter::set_viewbox( std::string viewbox ){
    this->viewbox = viewbox;
}

void SVGWriter::set_global_transform( std::string transform ){
    this->transform = transform;
}

void SVGWriter::write_prologue(){
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" ";
    svg << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n";
    if( viewbox.length() > 0 ){
        svg << "<svg viewBox=\"";
        svg << viewbox << "\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" ";
    }else{
        svg << "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" ";
    }
    svg << "xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n";
    if( transform.length() > 0 )
        svg << "<g transform=\"" << transform << "\">\n";
}

void SVGWriter::write_epilogue(){
    if (transform.length() > 0)
        svg << "</g>\n";
    svg << "</svg>";
}

void SVGWriter::write_style_attribute( std::vector<std::string>& candidates ){
    svg << "style=\"" ;
    bool is_first = true;
    for( std::string name: candidates ){
        auto value = properties.find( name );
        if( value != properties.end() ) {
            if( !is_first )
                svg << "; ";
            svg << name << ":" << value->second;
            is_first = false;
        }
    }
    svg << "\"" ;
}

void SVGWriter::write_circle( double x, double y, double r ){
    svg << "<circle cx=\"" << x << "\" cy=\"" << y << "\"";
    svg << " r=\"" << r << "\" ";
    write_style_attribute( shape_properties );
    svg << " />\n";
}

void SVGWriter::write_rectangle( double x1, double y1, double width, double height ){
    svg << "<rect x=\"" << x1 << "\" y=\"" << y1 << "\"";
    svg << " width=\"" << width << "\" height=\"" << height << "\" ";
    write_style_attribute( shape_properties );
    svg << " />\n";
}

void SVGWriter::write_segment( double x1, double y1, double x2, double y2 ){
    svg << "<line x1=\"" << x1 << "\" y1=\"" << y1 << "\" ";
    svg << "x2=\"" << x2 << "\" y2=\"" << y2 << "\" ";
    write_style_attribute( shape_properties );
    svg << " />\n";
}

void SVGWriter::write_image( double x1, double y1, double width, double height,
                             std::string format, std::string content_base64 ){
    svg << "<image x=\"" << x1 << "\" y=\"" << y1 << "\"";
    svg << " width=\"" << width << "\" height=\"" << height << "\"";
    svg << " style=\"image-rendering: pixelated\"";
    svg << " xlink:href=\"data:" << format << ";base64," << content_base64 << "\"/>\n";
}

std::vector<std::string> SVGWriter::shape_properties = {
        "fill",
        "stroke",
        "stroke-width"
};

} //namespace svg