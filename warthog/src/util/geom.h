#ifndef WARTHOG_GEOM_H
#define WARTHOG_GEOM_H

// util/geom.h
//
// miscellaneous collection of geometric things
//
// @author: dharabor
// @created 2016-07-31
//

#include <istream>
#include <ostream>
#include <float.h>

namespace warthog
{
namespace geom
{

struct rectangle
{
    public:
        int32_t x1, y1, x2, y2;

    rectangle()
    { 
        clear();
    }

    rectangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
    {
        this->x1 = x1; this->y1 = y1; 
        this->x2 = x2; this->y2 = y2;
    }

    rectangle(const rectangle& other)
    {
        x1 = other.x1;
        x2 = other.x2;
        y1 = other.y1;
        y2 = other.y2;
    }


    rectangle&
    operator=(const rectangle& other)
    {
        x1 = other.x1;
        x2 = other.x2;
        y1 = other.y1;
        y2 = other.y2;
        return *this;
    }


    bool
    operator==(const rectangle& other)
    {
        return 
            x1 == other.x1 && x2 == other.x2 &&
            y1 == other.y1 && y2 == other.y2;
    }

    bool
    operator!=(const rectangle& other)
    {
        return !this->operator==(other);
    }


    int32_t
    get_width() { return abs(x2 - x1); }

    int32_t
    get_height() { return abs(y2 - y1); }

    uint64_t
    get_area() { return ((uint64_t)abs(x2 - x1)) * ((uint64_t)abs(y2 - y1)); }

    void
    grow(int32_t x, int32_t y)
    {
        if(x < x1) { x1 = x; }
        if(y < y1) { y1 = y; }
        if(x > x2) { x2 = x; }
        if(y > y2) { y2 = y; }
    }

    void
    grow(const warthog::geom::rectangle& r)
    {
        if(r.x1 < x1) { x1 = r.x1; }
        if(r.y1 < y1) { y1 = r.y1; }
        if(r.x2 > x2) { x2 = r.x2; } 
        if(r.y2 > y2) { y2 = r.y2; } 
    }

    bool 
    contains(int32_t x, int32_t y)
    {
        return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    }

    // we compute the intersection of two rectangles 
    // r1 and r2 as follows: 
    // (1) create a bounding box r3 that contains r1 and r2
    // call its area A_r3
    // (2) create a rectangle r4 with dimensions
    // width = r1.width + r2.width and 
    // height = r1.height + r2.height
    // Call its area A_r4
    //
    // If A_r3 > A_r4 there cannot be any overlap
    // If A_r3 < A_r4 r1 and r2 definitely overlap
    // If A_r3 == A_r4 then r1 and r2 touch
    //
    // If the area from (2) is less than the area 
    // of (1) there is no overlap. 
    // if there is no overlap. On the other hand
    // if the area from (1) is smaller, the rectangles 
    // must overlap 
    bool 
    intersects(warthog::geom::rectangle& r)
    {
        rectangle r3(*this);
        r3.grow(r);

        rectangle r4;
        r4.x1 = r3.x1;
        r4.y1 = r3.y1;
        r4.y2 = r4.y1 + ((int32_t)this->get_height() + (int32_t)r.get_height());
        r4.x2 = r4.x1 + ((int32_t)this->get_width() + (int32_t)r.get_width());

        uint64_t r3_area = r3.get_area();
        uint64_t r4_area = r4.get_area();
        if(r3_area <= r4_area) { return true; }
        return false;
    }

    bool 
    is_valid() { return x2 >= x1 && y2 >= y1; }

    void
    print(std::ostream& out)
    {
        out << " rect " << x1 << "\t" << y1 << "\t"
            << x2 << "\t" << y2 <<std::endl;
    }

    void
    clear()
    {
        x1 = y1 = INT32_MAX;
        x2 = y2 = INT32_MIN;
    }
};

std::ostream&
operator<<(std::ostream& out, warthog::geom::rectangle& rect);

std::istream&
operator>>(std::istream& in, warthog::geom::rectangle& rect);

}


}

#endif

