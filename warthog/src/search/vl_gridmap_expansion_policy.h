#ifndef WARTHOG_VL_GRIDMAP_EXPANSION_POLICY_H
#define WARTHOG_VL_GRIDMAP_EXPANSION_POLICY_H

// search/vl_gridmap_expansion_policy.h
//
// An expansion policy for square gridmaps with
// vertex costs. There are eight possible move
// actions: 4 cardinal and 4 diagonal with
// costs 1 and sqrt(2) respectively.
//
// The edge costs are weighted by the average of
// the vertices they connect. 
//
// Example:
//
// ab
// cd
//
// In the case of a cardial move from cell a to b, 
// we compute the dge weight by computing the average 
// of the the labels found at vertex a and vertex b.
// Diagonal moves are similar but we take the 
// average of four cells
//
//
// @author: dharabor
// @created: 2014-09-17
// @updated: 2018-11-09
//

#include "expansion_policy.h"
#include "search_node.h"
#include "labelled_gridmap.h"

#include <memory>

namespace warthog
{

class vl_gridmap_expansion_policy : public expansion_policy
{
	public:
		vl_gridmap_expansion_policy(warthog::vl_gridmap* map);
		virtual ~vl_gridmap_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

		virtual size_t
		mem()
		{
			return 
                expansion_policy::mem() + 
                sizeof(*this) + map_->mem();
		}

        virtual void
        get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);


	private:
		warthog::vl_gridmap* map_;
};

}

#endif

