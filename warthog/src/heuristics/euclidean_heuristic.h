#ifndef WARTHOG_EUCLIDEAN_HEURISTIC_H
#define WARTHOG_EUCLIDEAN_HEURISTIC_H

// euclidean_heuristic.h
//
// Straight-line heuristic for measuring distances in the plane.
//
// @author: dharabor
// @created: 2016-02-11
//
//

#include "constants.h"
#include "forward.h"

namespace warthog
{

typedef void (*xyFn)(uint32_t id, int32_t& x, int32_t& y);
class euclidean_heuristic
{
    public:
        euclidean_heuristic(warthog::graph::xy_graph* g);
        ~euclidean_heuristic();

        double
        h(sn_id_t id, sn_id_t id2);

        static double
        h(double x, double y, double x2, double y2);

        void
        set_hscale(double hscale);

        double
        get_hscale();

        size_t
        mem(); 

        private:
        warthog::graph::xy_graph* g_;
        double hscale_;

};

}

#endif

