#ifndef WARTHOG_BB_LABELLING_H
#define WARTHOG_BB_LABELLING_H

// label/bb_labelling.h
//
// For every outgoing edge of every node in a graph we store a rectangular
// bounding box. Inside the box can be found all nodes that are reached 
// optimally by a path whose first edge is the edge at hand.
// The implementation is based on descriptions in the following paper:
//
// [Wager & Willhalm, 2005, 
// Geometric Containers for Efficient Shortest Path Computation, 
// Journal of Experimental Algorithms, vol 10, pp 1-30]
//
// @author: dharabor
// @created: 2020-03-14
//

#include "contraction/contraction.h"
#include "util/geom.h"
#include "util/timer.h"
#include "sys/forward.h"

#include <cassert>
#include <fstream>
#include <vector>
#include <cstdint>

namespace warthog
{

namespace label
{

struct bb_label
{
    typedef uint8_t T_FLAG;
    bb_label() { } 

    bb_label&
    operator=(const bb_label& other)
    {
        bbox_ = other.bbox_;
        return *this;
    }

    void
    merge(const bb_label& other)
    {
        bbox_.grow(other.bbox_);
    }

    void
    print(std::ostream& out)
    {
        out << " bb_label";
        bbox_.print(out);
    }
    warthog::geom::rectangle bbox_;
};

std::istream&
operator>>(std::istream& in, warthog::label::bb_label& label);

std::ostream&
operator<<(std::ostream& out, warthog::label::bb_label& label);

class bb_labelling 
{
    friend std::ostream&
    operator<<(std::ostream& out, bb_labelling& lab);

    friend std::istream&
    operator>>(std::istream& in, warthog::label::bb_labelling& lab);

    public:
    
        bb_labelling(warthog::graph::xy_graph* g);
        ~bb_labelling();

        inline warthog::graph::xy_graph*
        get_graph() 
        { 
            return g_;
        }

        bb_label&
        get_label(uint32_t node_id, uint32_t edge_idx)
        {
            assert(edge_idx < lab_->at(node_id).size());
            return lab_->at(node_id).at(edge_idx);
        }

        inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            for(uint32_t i = 0; i < lab_->size(); i++)
            {
                retval += (sizeof(bb_label) ) 
                    * lab_->at(i).size();
            }
            return retval;
        }

        // compute labels for all nodes specified by the given workload
        void
        precompute(warthog::util::workload_manager* workload);

    private:
        warthog::graph::xy_graph* g_;
        std::vector< std::vector< bb_label >>* lab_;
};

std::istream&
operator>>(std::istream& in, warthog::label::bb_labelling& lab);

std::ostream&
operator<<(std::ostream& in, warthog::label::bb_labelling& lab);

}

}

#endif

