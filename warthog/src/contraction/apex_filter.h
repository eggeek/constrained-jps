#ifndef WARTHOG_APEX_FILTER_H
#define WARTHOG_APEX_FILTER_H

// contraction/apex_filter.h
//
// a filter for forward contraction hierarchies in cases where the
// apex of the path is known (or when its rank can be upper-bounded):
//
// @author: dharabor
// @created: 2017-05-10
//

#include "constants.h"
#include "xy_graph.h"

#include <vector>
#include <cstdint>

namespace warthog
{

// allows passing apex-related data to certain expanders that support it
class apex_filter
{
    public:

        apex_filter(
                std::vector<uint32_t>* order, 
                warthog::graph::xy_graph* g)
        {
            g_ = g;
            apex_id_ = warthog::INF32;
            apex_reached_ = false;
            order_ = order;
            prune_after_apex_ = true;
            prune_before_apex_ = true;
        }

        inline bool
        filter(uint32_t node_id, uint32_t edge_idx)
        {
            // have we reached the apex yet?
            if(node_id == apex_id_)
            { apex_reached_ = true; }

            warthog::graph::node* n = g_->get_node(node_id);
            warthog::graph::edge* e = n->outgoing_begin() + edge_idx;

            uint32_t n_rank = order_->at(node_id);
            uint32_t s_rank = order_->at(e->node_id_);
            if(apex_reached_)
            {
                // never follow up edges after the apex is reached
                if((s_rank > n_rank) && prune_after_apex_) { return true; }
                return false;
            }
            
            // never follow down edges before the apex is reached
            if((s_rank < n_rank) && prune_before_apex_) { return true; }


            // always follow up edges before the apex is reached
            return false;
        }

        inline void
        set_apex(uint32_t node_id)
        {
            apex_id_ = node_id;
            apex_reached_ = false;
        }

        inline uint32_t
        get_apex() { return apex_id_; }
        
        bool prune_before_apex_;
        bool prune_after_apex_;
    
    private:
        uint32_t apex_id_;
        bool apex_reached_;
        std::vector<uint32_t>* order_;
        warthog::graph::xy_graph* g_;
};
}

#endif
