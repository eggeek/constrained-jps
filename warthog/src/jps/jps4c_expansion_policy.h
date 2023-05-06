#ifndef WARTHOG_JPS4C_EXPANSION_POLICY_H
#define WARTHOG_JPS4C_EXPANSION_POLICY_H

// jps/jps4c_expansion_policy.h
//
// Successor generating functions for Jump Point Search on 4-connected gridmaps
//
// @author: dharabor
// @created: 2019-11-13
//

#include "expansion_policy.h"
#include "jps/four_connected_jps_locator.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "online_jump_point_locator.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{

class jps4c_expansion_policy : public expansion_policy
{
	public:
		jps4c_expansion_policy(warthog::gridmap* map);
		virtual ~jps4c_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

		virtual inline size_t
		mem()
		{
            return expansion_policy::mem() +
                sizeof(*this) + map_->mem() + jpl_->mem();
		}

	private:
		warthog::gridmap* map_;
		warthog::four_connected_jps_locator* jpl_;

        warthog::jps::direction
        compute_direction(uint32_t n1_id, uint32_t n2_id);
};

}

#endif

