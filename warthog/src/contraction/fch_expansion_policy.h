#ifndef WARTHOG_FCH_EXPANSION_POLICY_H
#define WARTHOG_FCH_EXPANSION_POLICY_H

// contraction/fch_expansion_policy.h
//
// An expansion policy for forward-driven
// search in contraction hiearchies.
//
// The idea is simple:
// When expanding a node, look at the level
// of the parent relative to the current node.
//
// If the parent is smaller the search is traveling up in the hierarchy and 
// every neighbour is generated.
//
// If the parent is larger the search is traveling down in the hiearchy
// and only down-ward neighbours are generated.
//
// The approach preserves optimality. 
//
// @author: dharabor
// @created: 2016-07-18
//

#include "contraction.h"
#include "expansion_policy.h"
#include <vector>

namespace warthog
{

class fch_expansion_policy : public expansion_policy
{
    public:
        fch_expansion_policy(warthog::ch::ch_data* chd);

        virtual 
        ~fch_expansion_policy() { }

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
            return expansion_policy::mem() +
                sizeof(this);
        }

        warthog::ch::ch_data*
        get_ch_data() { return chd_; } 

    private:
        warthog::ch::ch_data* chd_;

        inline uint32_t
        get_level(warthog::sn_id_t id)
        {
            return chd_->level_->at(id);
        }
};
}

#endif
