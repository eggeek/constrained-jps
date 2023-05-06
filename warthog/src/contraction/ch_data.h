#ifndef WARTHOG_CH_CH_DATA_H
#define WARTHOG_CH_CH_DATA_H

// contraction/ch_data.h
//
// A contraction hierarchy is an augmented graph plus some
// additional data specifying the contraction order (== level)
// of each node.
//
// We further distinguish between contraction hierarchies
// optimised for bi-directional search and those which
// could be used in a forward search setting. The difference
// is how arcs are stored.
//
// In a forward contraction hierarchy we store all outgoing
// edges, whether up or down. This allows traversal of the
// hierarchy using a forward-only search algorithm such as A*.
//
// In a bi-directional contraction hierarchy we store outgoing
// up edges and incoming down edges. The outgoing edges are
// intended to be traversed with a bi-directional algorithm
// in the forward direction while the incoming down edges are
// intended to be traversed in the backward direction.
// From the perspective of the backward search, the incoming
// edges are also going up.
//
// @author: dharabor
// @created: 2019-xx-xx
//

#include "xy_graph.h"

namespace warthog
{
namespace ch
{

typedef enum
{
   UP_DOWN = 0,
   UP_ONLY = 1
} ch_type;

class ch_data
{

public:
    ch_data(bool store_incoming_edges=false)
    {
        g_ = new warthog::graph::xy_graph(0,"",store_incoming_edges);
        level_ = new std::vector<uint32_t>();
        up_degree_ = new std::vector<uint32_t>();
        type_ = warthog::ch::ch_type::UP_DOWN;
    }

    virtual ~ch_data()
    {
        up_degree_->clear();
        delete up_degree_;
        up_degree_ = 0;

        level_->clear();
        delete level_;
        level_ = 0;

        delete g_;
        g_ = 0;
    }

    size_t
    mem()
    {
        return
            g_->mem() +
            sizeof(uint32_t)*level_->size() +
            sizeof(uint32_t)*up_degree_->size() +
            sizeof(this);
    }

    warthog::graph::xy_graph* g_;
    std::vector<uint32_t>* level_;
    std::vector<uint32_t>* up_degree_;
    warthog::ch::ch_type type_;
};

std::ofstream&
operator<<(std::ofstream&, warthog::ch::ch_data& chd);

std::ifstream&
operator>>(std::ifstream&, warthog::ch::ch_data& chd);

}
}

#endif
