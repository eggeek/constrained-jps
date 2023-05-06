#ifndef WARTHOG_LL_EXPANSION_POLICY_H
#define WARTHOG_LL_EXPANSION_POLICY_H

// mapf/ll_expansion_policy.h
//
// An expansion policy for low-level search on uniform-cost manhattan grids.
// In this policy constraints are applied only on edges.

// At each location there are two different types of actions:
// - cardinal moves (at a cost of 1)
// - wait moves (at a cost of 1)
//
// Each action, except wait, moves the agent from its current grid
// location to an adjacent grid location. Each action (including wait)
// advances time by one time-step.
//
// @author: dharabor
// @created: 2018-11-01
//

#include "cbs_ll_heuristic.h"
#include "expansion_policy.h"
#include "forward.h"
#include "gridmap.h"
#include "search_node.h"
#include "time_constraints.h"

#include <memory>

namespace warthog
{

class ll_expansion_policy 
{
	public:
		ll_expansion_policy(
                warthog::gridmap* map, 
                warthog::cbs_ll_heuristic* h);

		~ll_expansion_policy();

		inline void
		reset()
		{
			current_ = 0;
            neis_->clear();
		}

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            current_ = 0;
            n(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(current_ < neis_->size())
            {
                ret = (*neis_)[current_].first;
                cost = (*neis_)[current_].second;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            current_++;
            n(ret, cost);
		}

		void 
		expand(warthog::search_node*, warthog::problem_instance*);

        void
        get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y);

        warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);
        
        warthog::search_node*
        generate(warthog::sn_id_t node_id)
        {
            return __generate((uint32_t)node_id, node_id>>32);
        }

        inline bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            // agents must arrive at the xy location of the target
            uint32_t xy_id = (uint32_t)(n->get_id() & UINT32_MAX);
            if(xy_id != (uint32_t)pi->target_id_) { return false; }

            uint32_t arrival_time = (uint32_t)(n->get_id() >> 32);
            std::vector<warthog::mapf::cell_constraint>& xy_cons = 
                cons_->get_constraint_set((uint32_t)(n->get_id()));

            // the arrival is _safe_ if the agent never has to move again
            for(uint32_t i = 0; i < xy_cons.size(); i++)
            {
                if((xy_cons.at(i).timestep_ >= arrival_time) &&
                    xy_cons.at(i).v_) 
                { return false; }
            }
            return true;
        }

        warthog::mapf::time_constraints<warthog::mapf::cell_constraint>*
        get_time_constraints() { return cons_; }

		size_t 
        mem();

	
	private:
        typedef std::pair<warthog::search_node*, double> neighbour_record;

		warthog::gridmap* map_;
        uint32_t map_xy_sz_;
        warthog::cbs_ll_heuristic* h_;

        std::vector<warthog::mem::node_pool*>* search_node_pool_;
        warthog::mapf::time_constraints<warthog::mapf::cell_constraint>* cons_;

        arraylist<neighbour_record>* neis_;
        uint32_t current_;

        inline warthog::search_node* 
        __generate(uint32_t xy_id, uint32_t timestep)
        {
            while(timestep >= search_node_pool_->size())
            {
                search_node_pool_->push_back(
                            new warthog::mem::node_pool(
                                map_->height() * map_->width()));
            }
            warthog::search_node* nei = search_node_pool_->at(timestep)->generate(xy_id);
            nei->set_id((((warthog::sn_id_t)timestep) << 32) | xy_id);
            return nei;
        }


        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
        }

};

}

#endif

