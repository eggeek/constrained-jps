#ifndef WARTHOG_DEPTH_FIRST_SEARCH_H
#define WARTHOG_DEPTH_FIRST_SEARCH_H

// search/depth_first_search.h
//
// An implementation of depth-first search with some extras including:
// - cost cutoffs
// - expansion cutoffs
// - expansion event callbacks
// - heuristic function support
//
// NB: This search is intended for POLYNOMIAL DOMAINS. where the total
// number of nodes in the search space does not grow exponentially
// with the size of the problem instance.
// For exponential domains, the memory allocator needs to be different
// and we need a hashing container to track which nodes are on the 
// current branch (i.e. to avoid cycles).
//
// @author: dharabor
// @created: 2020-03-02
//

#include "cpool.h"
#include "pqueue.h"
#include "problem_instance.h"
#include "search.h"
#include "search_node.h"
#include "solution.h"
#include "timer.h"

#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace warthog
{

// H is a heuristic function
// E is an expansion policy
// Q is data structure that determines the order of DFS successors
template< class H, 
          class E, 
          class Q = warthog::pqueue_min >
class depth_first_search : public warthog::search
{
    typedef std::pair<warthog::search_node*, uint32_t> dfs_pair;

	public:
		depth_first_search(H* heuristic, E* expander, Q* queue)
            : heuristic_(heuristic), expander_(expander), succ_queue_(queue)
		{
            cost_cutoff_ = warthog::COST_MAX;
            exp_cutoff_ = UINT32_MAX;
            on_generate_fn_ = 0;
            on_expand_fn_ = 0;
            pi_.instance_id_ = UINT32_MAX;
            stack_.reserve(4096);
		}

		virtual ~depth_first_search() { }

        virtual void
		get_pathcost(
                warthog::problem_instance& instance, warthog::solution& sol)
        {

            sol.reset();
            pi_ = instance;

			warthog::search_node* target = search(sol);
			if(target)
			{
                sol.sum_of_edge_costs_ = target->get_g();
            }
        }

        virtual void
		get_path(warthog::problem_instance& instance, warthog::solution& sol)
		{
            sol.reset();
            pi_ = instance;

			warthog::search_node* target = search(sol);
			if(target)
			{
                sol.sum_of_edge_costs_ = target->get_g();

				// follow backpointers to extract the path
				assert(expander_->is_target(target, &pi_));
                warthog::search_node* current = target;
				while(true)
                {
                    sol.path_.push_back(current->get_id());
                    if(current->get_parent() == warthog::SN_ID_MAX) break;
                    current = expander_->generate(current->get_parent());
				}
                std::reverse(sol.path_.begin(), sol.path_.end());

                #ifndef NDEBUG
                if(pi_.verbose_)
                {
                    for(auto& state : sol.path_)
                    {
                        int32_t x, y;
                        expander_->get_xy(state, x, y);
                        std::cerr 
                            << "final path: (" << x << ", " << y << ")...";
                        warthog::search_node* n = expander_->generate(state);
                        assert(n->get_search_number() == pi_.instance_id_);
                        n->print(std::cerr);
                        std::cerr << std::endl;
                    }
                }
                #endif
            }
		}

        // apply @param fn every time a node is generated (equiv, reached)
        void
        apply_on_generate( 
                std::function<void( 
                    warthog::search_node* succ, 
                    warthog::search_node* from, 
                    warthog::cost_t edge_cost, 
                    uint32_t edge_id)>& fn)
        {
            on_generate_fn_ = &fn;
        }

        // apply @param fn when a node is popped off the stack for expansion
        void
        apply_on_expand(std::function<void(warthog::search_node*)>& fn)
        {
            on_expand_fn_ = &fn;
        }

        // set a cost-cutoff to run a bounded-cost search.  the search 
        // never expands a node whose f-value is more than the limit.
        inline void
        set_cost_cutoff(warthog::cost_t cutoff) { cost_cutoff_ = cutoff; }

        inline warthog::cost_t
        get_cost_cutoff() { return cost_cutoff_; }

        // set a cutoff on the maximum number of node expansions.
        // the search terminates when the limit is reached, unless the target
        // is expanded sooner.
        inline void
        set_max_expansions_cutoff(uint32_t cutoff) { exp_cutoff_ = cutoff; }

        inline uint32_t 
        get_max_expansions_cutoff() { return exp_cutoff_; }  

		virtual inline size_t
		mem()
		{
			size_t bytes = 
				sizeof(dfs_pair)*stack_.size() + 
				expander_->mem() +
                heuristic_->mem() +
				sizeof(*this);
			return bytes;
		}


	private:
		H* heuristic_;
		E* expander_;
		Q* succ_queue_;
        warthog::problem_instance pi_;

        std::vector<dfs_pair> stack_;

        // search parameters 
        warthog::cost_t cost_cutoff_; 
        uint32_t exp_cutoff_;

        // callback for when a node is expanded
        std::function<void(warthog::search_node*)>* on_expand_fn_;
        //
        // callback for when a node is reached / generated
        std::function<void(
                warthog::search_node*, 
                warthog::search_node*, 
                warthog::cost_t edge_cost, 
                uint32_t edge_id)>* on_generate_fn_;

		// no copy ctor
		depth_first_search(const depth_first_search& other) { } 

        // no assignment operator
		depth_first_search& 
		operator=(const depth_first_search& other) { return *this; }

		warthog::search_node*
		search(warthog::solution& sol)
		{
			warthog::timer mytimer;
			mytimer.start();
			stack_.clear();

			warthog::search_node* start;
			warthog::search_node* target = 0;

            // get the internal target id
            if(pi_.target_id_ != warthog::SN_ID_MAX)
            { 
                warthog::search_node* target = 
                    expander_->generate_target_node(&pi_);
                if(!target) { return 0; } // invalid target location
                pi_.target_id_ = target->get_id();

            }

            // initialise and push the start node
            if(pi_.start_id_ == warthog::SN_ID_MAX) { return 0; }
            start = expander_->generate_start_node(&pi_);
            if(!start) { return 0; } // invalid start location
            pi_.start_id_ = start->get_id();

			start->init(pi_.instance_id_, warthog::SN_ID_MAX, 
                    0, heuristic_->h(pi_.start_id_, pi_.target_id_));
			stack_.push_back(dfs_pair(start, 0));
            sol.nodes_inserted_++;

            
            if(on_generate_fn_) 
            { (*on_generate_fn_)(start, 0, 0, UINT32_MAX); }



			#ifndef NDEBUG
			if(pi_.verbose_) { pi_.print(std::cerr); std:: cerr << "\n";}
			#endif

            // begin expanding
			while(stack_.size())
			{
                // terminate if we've reached the limit for expanded nodes
                if(sol.nodes_expanded_ > exp_cutoff_) { break; }

                // search continues; pop a node off the stack
                dfs_pair& c_pair = stack_.back();
				warthog::search_node* current = c_pair.first;

                // goal test
                if(expander_->is_target(current, &pi_))
                {
                    target = current;
                    break;
                }

                // we process successors in order, from first to last
                // as they come out of the expansion policy
				expander_->expand(current, &pi_);
                current->set_expanded(true);
				sol.nodes_expanded_++;
                if(on_expand_fn_) { (*on_expand_fn_)(current); }

				#ifndef NDEBUG
				if(pi_.verbose_)
				{
					int32_t x, y;
                    expander_->get_xy(current->get_id(), x, y);
					std::cerr 
                        << sol.nodes_expanded_
                        << ". expanding ("<<x<<", "<<y<<")...";
					current->print(std::cerr);
					std::cerr << std::endl;
				}
				#endif


                // enumerate successors
				warthog::search_node* n = 0;
				warthog::cost_t cost_to_n = 0;
                expander_->get_successor(c_pair.second++, n, cost_to_n);

                if(n == 0)
                {
                    // all successors exhausted. backtrack
                    //current->set_expanded(false); 
                    stack_.pop_back();
                    continue;
                }

                for( ; n != 0; expander_->next(n, cost_to_n) )
                {
                    // to avoid cycles we store some data that records whether
                    // or not the proposed successor appears on the current branch
                    if( n->get_expanded() && 
                        n->get_search_number() == current->get_search_number() )
                    {
                        continue;
                    }

                    warthog::cost_t gval = current->get_g() + cost_to_n;
                    n->init(current->get_search_number(), current->get_id(),
                        gval, gval + heuristic_->h(n->get_id(),pi_.target_id_));

                    // only generate successors below the f-value threshold
                    if(n->get_f() < cost_cutoff_) 
                    { 
                        stack_.push_back(dfs_pair(n, 0));
                        sol.nodes_inserted_++;
                        if(on_expand_fn_) { (*on_expand_fn_)(n); }

                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            int32_t nx, ny;
                            expander_->get_xy(n->get_id(), nx, ny);
                            std::cerr 
                                << "  generating (edgecost=" 
                                << cost_to_n<<") ("<< nx <<", "<< ny <<")...";
                            n->print(std::cerr);
                            std::cerr << std::endl;
                        }
                        #endif
                        break;
                    }

                }
			}

			mytimer.stop();
			sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                if(target == 0) 
                {
                    std::cerr 
                        << "search failed; no solution exists " << std::endl;
                }
                else
                {
                    int32_t x, y;
                    expander_->get_xy(target->get_id(), x, y);
                    std::cerr << "target found ("<<x<<", "<<y<<")...";
                    target->print(std::cerr);
                    std::cerr << std::endl;
                }
            }
            #endif

            return target;
		}

}; // depth_first_search

} // warthog

#endif
