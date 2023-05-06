#pragma once
// jps2_expansion_policy_prune2.h

// @author: shizhe
// @created: 30/06/2021

#include "node_pool.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "expansion_policy.h"
#include "online_jump_point_locator2_prune2.h"
#include "problem_instance.h"
#include "search_node.h"
#include "online_jps_pruner2.h"

#include "stdint.h"

namespace warthog
{

class jps2_expansion_policy_prune2: public expansion_policy
{
	public:
		jps2_expansion_policy_prune2(warthog::gridmap* map);
		~jps2_expansion_policy_prune2();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

		virtual inline size_t
		mem()
		{
			return expansion_policy::mem() + 
                sizeof(*this) + map_->mem() + jpl_->mem();
		}

    virtual void
    get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y); 

    virtual warthog::search_node* 
    generate_start_node(warthog::problem_instance* pi);

    virtual warthog::search_node*
    generate_target_node(warthog::problem_instance* pi);

    warthog::online_jump_point_locator2_prune2* get_locator() {
      return this->jpl_;
    }

    void init_tables() {
      this->jpl_->init_tables();
    }

    // set loc to be empty(empty=true) or blocked(empty=false)
    inline void perturbation(sn_id_t loc, bool empty) {
      warthog::gridmap* mapptr = jpl_->get_map();
      warthog::gridmap* rmapptr = jpl_->get_rmap();
      mapptr->set_label(loc, empty);
      // map id to rmap id
      uint32_t x, y, rx, ry;
      mapptr->to_unpadded_xy(loc, x, y);
      ry = x, rx = mapptr->header_height() - y - 1;
      sn_id_t rloc = rmapptr->to_padded_id(rx, ry);
      rmapptr->set_label(rloc, empty);
    }

	private:
		warthog::gridmap* map_;
		online_jump_point_locator2_prune2* jpl_;
		std::vector<warthog::cost_t> costs_;
		std::vector<uint32_t> jp_ids_;
    online_jps_pruner2 jpruner;

    inline warthog::jps::direction compute_direction (
            uint32_t n1_id, uint32_t n2_id)
    {
        if(n1_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

        int32_t x, y, x2, y2;
        warthog::helpers::index_to_xy(n1_id, map_->width(), x, y);
        warthog::helpers::index_to_xy(n2_id, map_->width(), x2, y2);
        int32_t dx = abs(x2 - x);
        int32_t dy = abs(y2 - y);

        if(dx > dy)
        {
            if(x2 > x)
            { return warthog::jps::EAST; }

            return warthog::jps::WEST;
        }

        if(y2 > y) 
        { return warthog::jps::SOUTH; }

        return warthog::jps::NORTH;
    }
};

}
