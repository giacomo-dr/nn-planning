// svg_writer.h
// Author: Giacomo Del Rio
// Date: 20 Apr 2017

#ifndef SVGWRITER_H
#define SVGWRITER_H

#include <string>
#include <fstream>
#include <map>
#include <vector>

namespace svg {

class SVGWriter {
public:
    SVGWriter( std::string filename );
    virtual ~SVGWriter();

    void begin();
    void end();
    void set_style_property( std::string name, std::string value );
    void clear_style_property( std::string name );
    void set_viewbox( std::string viewbox );
    void set_global_transform( std::string transform );
    void write_circle( double x, double y, double r = 5 );
    void write_rectangle( double x1, double y1, double width, double height );
    void write_segment( double x1, double y1, double x2, double y2 );
    void write_image( double x1, double y1, double width, double height,
                      std::string format, std::string content_base64 );
    //void writePolygon(const Polygon& p);
    //void writePolygons(const vector<Polygon>& pv);
    //void writeRectangle(const Rectangle& r);
    //void writePolyline(const vector<IntPoint2D>& pl);

private:
    void write_prologue();
    void write_epilogue();
    void write_style_attribute( std::vector<std::string>& candidates );

private:
    std::string filename;
    std::ofstream svg;

    std::string viewbox;
    std::string transform;
    std::map<std::string, std::string> properties;

    static std::vector<std::string> shape_properties;
};

} //namespace svg


#endif //SVGWRITER_H
