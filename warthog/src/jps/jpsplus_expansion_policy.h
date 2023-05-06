#ifndef WARTHOG_JPSPLUS_EXPANSION_POLICY_H
#define WARTHOG_JPSPLUS_EXPANSION_POLICY_H

// jpsplus_expansion_policy.h
//
// JPS+ is Jump Point Search together with a preprocessed database 
// that stores all jump points for every node.
//
// Theoretical details:
// [Harabor and Grastien, 2012, The JPS Pathfinding System, SoCS]
//
// @author: dharabor
// @created: 05/05/2012

#include "expansion_policy.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "offline_jump_point_locator.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{

class jpsplus_expansion_policy : public expansion_policy
{
	public:
		jpsplus_expansion_policy(warthog::gridmap* map);
		virtual ~jpsplus_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

		virtual inline size_t
		mem()
		{
			return 
                expansion_policy::mem() +
                sizeof(*this) + 
                map_->mem() + jpl_->mem();
		}

        virtual void
        get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

	private:
		warthog::gridmap* map_;
		offline_jump_point_locator* jpl_;

		// computes the direction of travel; from a node n1
		// to a node n2.
		inline warthog::jps::direction
		compute_direction(uint32_t n1_id, uint32_t n2_id)
		{
			if(n1_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

			int32_t x, y, x2, y2;
			warthog::helpers::index_to_xy(n1_id, map_->width(), x, y);
			warthog::helpers::index_to_xy(n2_id, map_->width(), x2, y2);
			warthog::jps::direction dir = warthog::jps::NONE;
			if(y2 == y)
			{
				if(x2 > x)
					dir = warthog::jps::EAST;
				else
					dir = warthog::jps::WEST;
			}
			else if(y2 < y)
			{
				if(x2 == x)
					dir = warthog::jps::NORTH;
				else if(x2 < x)
					dir = warthog::jps::NORTHWEST;
				else // x2 > x
					dir = warthog::jps::NORTHEAST;
			}
			else // y2 > y 
			{
				if(x2 == x)
					dir = warthog::jps::SOUTH;
				else if(x2 < x)
					dir = warthog::jps::SOUTHWEST;
				else // x2 > x
					dir = warthog::jps::SOUTHEAST;
			}
			assert(dir != warthog::jps::NONE);
			return dir;
		}
};

}

#endif

