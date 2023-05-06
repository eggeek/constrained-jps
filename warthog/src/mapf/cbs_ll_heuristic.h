#ifndef WARTHOG_CBS_LL_HEURISTIC_H
#define WARTHOG_CBS_LL_HEURISTIC_H

// mapf/cbs_ll_heuristic.h
//
// The low-level (i.e. single-agent) heuristic function used in 
// Conflict-based Search. Pre-computes all distances, from every 
// target node to every other node in the input graph. 
//
// This implementation assumes the input graph is a 4-connected 
// uniform-cost grid. 
//
// For more details see: 
// Sharon, Guni, et al. "Conflict-based search for optimal multi-agent pathfinding." 
// Artificial Intelligence 219 (2015): 40-66.
//
// @author: dharabor
// @created: 2018-11-04
//

#include "forward.h"
#include "gridmap.h"
#include "pqueue.h"
#include <unordered_map>

namespace warthog
{

class cbs_ll_heuristic
{
    public:
        cbs_ll_heuristic(warthog::gridmap* gm);

        ~cbs_ll_heuristic();

        // estimate the cost-to-go of a path that begins at 
        // node @param p_from_id and finishes at node @param p_to_id
        // NB: Both parameters need to be PADDED IDENTIFIERS
        inline warthog::cost_t
        h(warthog::sn_id_t p_from_id, warthog::sn_id_t p_to_id)
        {
            return h_[t_index_][(uint32_t)p_from_id];
        }

        // The current target specifies which set of g-values to
        // refer to when answering ::h queries
        // 
        // If the target hasn't been seen before, we run a Dijkstra grid 
        // search (not time expanded!) using @param target_id as the  source
        // 
        // We store g-values from @param target_id to every node in the grid.
        // These precomputed g-values are a lower-bound on the
        // distance from any location@time to each target.
        //
        // @param target_id: unpadded xy index specifying the current target
        // 
        // NB: @param target_nodes must comprise UNPADDED IDENTIFIERS
        void
        set_current_target(warthog::sn_id_t target_id);

        size_t
        mem();

    private:

        // callback listener used to construct a perfect 2D heuristic
        struct listener
        {
            listener(cbs_ll_heuristic* ptr)
            { this->ptr = ptr; }

            inline void
            generate_node(warthog::search_node* parent, 
                          warthog::search_node* child, 
                          warthog::cost_t edge_cost,
                          uint32_t edge_id) { } 

            inline void
            expand_node(warthog::search_node* current) 
            {
                ptr->h_[ptr->t_index_][(uint32_t)current->get_id()] = 
                    current->get_g();
            }

            inline void
            relax_node(warthog::search_node* current) { }

            cbs_ll_heuristic* ptr;
        };


        // things we need to compute a perfect heuristic
        warthog::solution* sol_;
        warthog::pqueue_min* open_;
        warthog::zero_heuristic* zh_;
        warthog::gridmap_expansion_policy* expander_;
        warthog::cbs_ll_heuristic::listener* listener_;

        warthog::flexible_astar< 
            warthog::zero_heuristic, 
            warthog::gridmap_expansion_policy,
            warthog::pqueue_min, 
            warthog::cbs_ll_heuristic::listener >* alg_;

        // things we need to store perfect heuristic values
        std::vector<std::vector<warthog::cost_t>> h_;
        std::unordered_map<uint32_t, uint32_t> t_map_;
        uint32_t t_index_;
        uint32_t gm_sz_;
};

}

#endif
