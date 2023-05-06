#ifndef WARTHOG_GRIDMAP_EXPANSION_POLICY_H
#define WARTHOG_GRIDMAP_EXPANSION_POLICY_H

// gridmap_expansion_policy.h
//
// An ExpansionPolicy for square uniform-cost grids.
// Supports octile movement and manhattan movement.
//
// In the case where diagonal moves are allowed
// corner-cutting is forbidden.
// i.e. 
//
// ab
// cd
//
// the move c -> b is only permitted if both 'a' and 'd'
// are traversable.
//
// @author: dharabor
// @created: 28/10/2010
//

#include "expansion_policy.h"
#include "gridmap.h"
#include "search_node.h"

#include <memory>

namespace warthog
{

class problem_instance;
class gridmap_expansion_policy : public expansion_policy
{
	public:
		gridmap_expansion_policy(warthog::gridmap* map, bool manhattan = false);
		virtual ~gridmap_expansion_policy() { }

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(sn_id_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

		virtual size_t
		mem();
	
	private:
		warthog::gridmap* map_;
        bool manhattan_;
};

}

#endif

