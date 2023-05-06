#ifndef WARTHOG_SIPP_EXPANSION_POLICY_H
#define WARTHOG_SIPP_EXPANSION_POLICY_H

// search/sipp_expansion_policy.h
// 
// An implementation of Safe Interval Path Planning. 
// For details see the following paper:
//
// [Phillips, M. and Likhachev, M., 2011, 
// SIPP: Safe interval path planning for Dynamic Environments. 
// In 2011 IEEE International Conference on Robotics and Automation 
// (pp. 5628-5635)]
// 
// This implementation of SIPP supports obstalces that have
// trajectories; i.e. each time an obstacle disappears we record
// the direction (if any) in which the obstacle moved
//
// @author: dharabor
// @created 2019-10-25
//

#include "sipp/sipp_gridmap.h"
#include "memory/node_pool.h"
#include "search/expansion_policy.h"
#include "search/search_node.h"

#include <vector>

namespace warthog
{

class sipp_expansion_policy
{
    public:
        sipp_expansion_policy(warthog::sipp_gridmap* gm);
        virtual ~sipp_expansion_policy(); 

        // get a search_node memory pointer associated with @param node_id. 
        // (value is null if @param node_id is bigger than nodes_pool_size_)
		inline warthog::search_node*
		generate(warthog::sn_id_t node_id)
		{
            warthog::sn_id_t xy_id = node_id & INT32_MAX;
            warthog::sn_id_t index = node_id >> 32;

            while(pool_.size() <= index)
            { pool_.push_back(new warthog::mem::node_pool(sz_xy)); }
            warthog::search_node* node = pool_.at(index)->generate(xy_id);
            node->set_id(node_id);
            return node;
		}

        // the start node is the first safe interval for the 
        // associated grid location. We require this interval
        // to start at time 0 
        inline warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi)
        {
            warthog::sipp::safe_interval start_si = 
                sipp_map_->get_safe_interval(
                    (uint32_t)pi->start_id_, 0);
            if(start_si.s_time_ != 0) { return 0; }
            return generate(pi->start_id_);
        }

        // the target node in SIPP is an xy location which 
        // the agent must reach. This function can be adjusted.
        // e.g. for some types of problems the target may be a 
        // at a specific time index or part of a specific 
        // safe interval. In MAPF the agent needs to wait 
        // indefinitely once it reaches its target 
        // location. This can only be satisfied if the safe
        // interval ends at time infinity.
        // 
        // NB: See also the function ::is_target where we need to
        // detect whether a node being expanded is the target
        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi)
        {
            uint32_t xy_id = (pi->target_id_ & INT32_MAX);
            if(!sipp_map_->gm_->get_label(sipp_map_->gm_->to_padded_id(xy_id)))
            {
                return 0; // target is an obstacle
            }
            return generate(xy_id);

            // ENABLE THIS CODE FOR MAPF TARGET CONDITION
            //std::vector<warthog::sipp::safe_interval>& ivals = 
            //    sipp_map_->get_all_intervals(xy_id);
            //for(uint32_t i = 0; i < ivals.size(); i++)
            //{
            //    // update the target identifier once we
            //    // find a suitable interval where the agent
            //    // can wait
            //    if(ivals.at(i).e_time_ == warthog::COST_MAX)
            //    {
            //        warthog::sn_id_t tmp_id = i;
            //        pi->target_id_ = (tmp_id << 32) + xy_id;
            //        return generate((warthog::sn_id_t)pi->target_id_);
            //    }
            //}
            //return 0; // no such target

        }

        // The objective in SIPP is to reach a specific xy location 
        // at the earliest possible time. This function returns true
        // if the node @param n corresponds to the xy location of the
        // target, at any timestep.
        //
        // NB: This function may need to be adjusted depending on the
        // problem at hand. In some settings, such as MAPF, the objective 
        // is to find the earliest time at which the target location can
        // be reached such that the agent can wait at the target
        // indefinitely (i.e. the safe interval must end at time INF)
        bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            uint32_t xy_id = (n->get_id() & INT32_MAX);
            return xy_id == pi->target_id_;

            //return n->get_id() == pi->target_id_;
        }


        // return the xy coordinates corresponding to the search node 
        // identifier @param node_id
        void
        get_xy(sn_id_t node_id, int32_t& x, int32_t& y) 
        {
            uint32_t xy_id = (uint32_t)(node_id & INT32_MAX);
            y = (int32_t)(xy_id / sipp_map_->gm_->header_width());
            x = (int32_t)(xy_id % sipp_map_->gm_->header_width());
        }

        // SIPP looks for successors among the set of safe intervals stored
        // with each xy location adjacent to that of @param current
        // Generated are all safe intervals which begin before the end (<=)
        // of the safe interval associated with @param current
		inline void 
		expand(warthog::search_node* current, warthog::problem_instance* problem)
        {
            reset();
            uint32_t c_xy_id = (uint32_t)(current->get_id() & INT32_MAX);
            uint32_t c_index = (uint32_t)(current->get_id() >> 32);
            warthog::sipp::safe_interval c_si = 
                sipp_map_->get_safe_interval(c_xy_id, c_index);
            assert(current->get_g() >= c_si.s_time_);
            assert(current->get_g() < c_si.e_time_);

            // get the neighbouring tiles
            uint32_t tiles = 0;
            sipp_map_->gm_->get_neighbours(
                sipp_map_->gm_->to_padded_id(c_xy_id), (uint8_t*)&tiles);

            if((tiles & 514) == 514) // N
            {  
                uint32_t succ_xy_id = c_xy_id - sipp_map_->gm_->header_width();
                generate_successors(current, c_si, succ_xy_id, warthog::cbs::move::SOUTH, problem);
            } 
            if((tiles & 1536) == 1536) // E
            {
                uint32_t succ_xy_id = c_xy_id + 1;
                generate_successors(current, c_si, succ_xy_id, warthog::cbs::move::WEST, problem);
            }
            if((tiles & 131584) == 131584) // S
            { 
                uint32_t succ_xy_id = c_xy_id + sipp_map_->gm_->header_width();
                generate_successors(current, c_si, succ_xy_id, warthog::cbs::move::NORTH, problem);
            }
            if((tiles & 768) == 768) // W
            { 
                uint32_t succ_xy_id = c_xy_id - 1;
                generate_successors(current, c_si, succ_xy_id, warthog::cbs::move::EAST, problem);
            }
        }

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
                ret = (*neis_)[current_].node_;
                cost = (*neis_)[current_].cost_;
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

        size_t
        mem()
        {
            size_t retval = 0;
            for(uint32_t i = 0; i < pool_.size(); i++)
            {
                retval += pool_.at(i)->mem();
            }
            retval += sipp_map_->mem();
            retval += sizeof(this);
            retval += sizeof(neighbour_record)*neis_->size();
            return retval;
        }

    private:
        warthog::sipp_gridmap* sipp_map_;
        uint32_t sz_xy;

        struct neighbour_record
        {
            neighbour_record(warthog::search_node* node, warthog::cost_t cost)
            {
                node_ = node;
                cost_ = cost;
            }
            warthog::search_node* node_;
            warthog::cost_t cost_;
        };
        arraylist<neighbour_record>* neis_;
        uint32_t current_;

        // we need to hash each sipp::safe_interval to a warthog::search_node 
        // and each warthog::search_node to a sipp::safe_interval
        // we use several pools of preallocated memory for this:
        // one pool for the set of sipp::safe_intervals which have index 0
        // one pool for the set of sipp::safe_intervals which have index 1
        // one pool for the set of sipp::safe_intervals which have index 2
        // ...
        // and so on
        // the upper 4 bytes of each warthog::search_node::id specifies
        // the (x, y) location of the node containing the sipp::safe_interval
        // the lower 4 bytes specifies the index of the sipp::safe_interval
        std::vector<warthog::mem::node_pool*> pool_;


        inline void 
        generate_successors(warthog::search_node* current, 
                warthog::sipp::safe_interval& c_si, 
                uint32_t succ_xy_id, warthog::cbs::move ec_direction,
                warthog::problem_instance* pi)
        {
            // iterate over adjacent safe intervals
            std::vector<warthog::sipp::safe_interval>& neis 
                = sipp_map_->get_all_intervals(succ_xy_id);

            for(uint32_t i = 0;
                         i < neis.size(); 
                         i++)
            {
                // we generate safe intervals for adjacent cells but:
                // (i) only if the successor safe interval begins before 
                // (i.e. <=) the end of the current safe interval and; 
                // (ii) only if the current safe interval is safe for
                // the duration of the action that moves the agent
                // (iii) only if the successor is safe at the time the 
                // agent finishes moving.
                warthog::sipp::safe_interval& succ_si = neis.at(i);

                warthog::cost_t action_cost = 1;
                if( succ_si.s_time_ <= c_si.e_time_ && 
                    current->get_g() < succ_si.e_time_)
                {
                    // if the adjacent safe interval begins at some time
                    // in the future then we wait at the current
                    // safe interval and move away at the earliest time
                    if(succ_si.s_time_ >= (current->get_g() + action_cost))
                    {
                        // the wait+move action completes exactly as 
                        // the successor interval begins
                        action_cost = succ_si.s_time_ - current->get_g();

                        // prune: avoid edge colllisions when moving
                        // into the successor interval
                        if(succ_si.action_ == ec_direction)
                        { continue; }
                    }

                    // prune: not enough time remains (current interval) 
                    // to execute the proposed action
                    if((current->get_g() + action_cost) > c_si.e_time_) 
                    { continue;  }

                    // prune: not enough time remains (successor interval) 
                    // to execute the proposed action
                    if((current->get_g() + action_cost) >= succ_si.e_time_) 
                    { continue;  }

                    // generate successor
                    warthog::sn_id_t succ_node_id = succ_xy_id;
                    succ_node_id = ((warthog::sn_id_t)i << 32) + succ_node_id;
                    add_neighbour(generate(succ_node_id), action_cost);
                }
            }
        }

        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
            //std::cout << " neis_.size() == " << neis_->size() << std::endl;
        }
};

}

#endif

