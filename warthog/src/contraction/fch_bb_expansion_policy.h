#ifndef WARTHOG_FCH_BB_EXPANSION_POLICY_H
#define WARTHOG_FCH_BB_EXPANSION_POLICY_H

// contraction/fch_bb_expansion_policy.h
//
// An expansion policy that combines Foward Search, Contraction Hierarchies
// and  Geometric Containers, implemented as rectangular bounding boxes. 
// To improve preprocessing times it is possible to precompute BB labels only 
// for some percentage of the highest nodes in the CH. 
// The rest of the nodes receive BB labels computed with depth-first search.
// This implementation is based on the following paper:
//
// [ Forward Search in Contraction Hierarchies. 2018. 
//   D. Harabor and P. Stuckey. 11th International 
//   Symposium on Combinatorial Search (SoCS) ] 
//
// @author: dharabor
// @created: 2017-12-02
//

#include "contraction.h"
#include "dfs_labelling.h"
#include "expansion_policy.h"
#include "forward.h"
#include "xy_graph.h"

#include <vector>

namespace warthog
{

class fch_bb_expansion_policy : public expansion_policy
{
    public:
        fch_bb_expansion_policy(warthog::label::dfs_labelling* lab);

        virtual 
        ~fch_bb_expansion_policy() { }

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        virtual inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            retval += chd_->mem(); 
            retval += expansion_policy::mem();
            return retval;
        }

    private:
        warthog::ch::ch_data* chd_;

        warthog::label::dfs_labelling* lab_;
        uint32_t s_label, t_label;
        int32_t tx_, ty_;
        uint32_t t_graph_id;
        uint32_t t_level;

        typedef bool
                (warthog::fch_bb_expansion_policy::*filter_fn)
                (uint32_t node_idx, uint32_t edge_idx);

        filter_fn filter;

        inline bool
        filter_all(uint32_t node_idx, uint32_t edge_idx)
        {
            warthog::label::dfs_label& label = 
                lab_->get_label(node_idx, edge_idx);
            bool retval = label.bbox_.contains(tx_, ty_);
            return !retval; 
        }

        inline bool
        filter_bb_only(uint32_t node_idx, uint32_t edge_idx)
        {
            warthog::label::dfs_label& label = 
                lab_->get_label(node_idx, edge_idx);
            bool retval = label.bbox_.contains(tx_, ty_);
            return !retval; 
        }

        inline uint32_t
        get_level(uint32_t id)
        {
            return chd_->level_->at(id);
        }
};
}

#endif
