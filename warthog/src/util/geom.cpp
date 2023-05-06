#include "geom.h"
#include <cassert>

std::ostream&
warthog::geom::operator<<(std::ostream& out, warthog::geom::rectangle& rect)
{
    out.write((char*)(&rect.x1), sizeof(rect.x1));
    out.write((char*)(&rect.x2), sizeof(rect.x2));
    out.write((char*)(&rect.y1), sizeof(rect.y1));
    out.write((char*)(&rect.y2), sizeof(rect.y2));
    return out;
}

std::istream&
warthog::geom::operator>>(std::istream& in, warthog::geom::rectangle& rect)
{
    in.read((char*)(&rect.x1), sizeof(rect.x1));
    in.read((char*)(&rect.x2), sizeof(rect.x2));
    in.read((char*)(&rect.y1), sizeof(rect.y1));
    in.read((char*)(&rect.y2), sizeof(rect.y2));
    return in;
}
