#ifndef ANYTIME_ASTAR_H
#define ANYTIME_ASTAR_H

// search/anytime_astar.h
//
// An anytime variant of A*. The implementation here is based on
// the class anytime_astar with some differences:
//      - There exists a time cutoff.
//      - The heuristic needs to implement an upperbound function
//        ::ub in addition to the lowerbounding function ::h.
//        The upperbound tells the remaining distance from a given 
//        node to the target along a known and concrete path. 
//      - The search terminates whenever 
//          (i) the time is exceeded or; 
//          (ii) the best node on the open list has a  gap 
//          (upperbound - lowerbound) of zero.
//
// On terminaton the algorithm returns the best known (incumbent) 
// path to the target.
//
// For more details see the following paper:
// [ E. Hansen and R. Zhou. 2007. Anytime Heuristic Search.
//   Journal of Artificial Intelligence Research (JAIR) ]
//
// @author: dharabor
// @created: 2020-03-04
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
// Q is min priority queue
template< class H, 
          class E, 
          class Q = warthog::pqueue_min >
class anytime_astar : public warthog::search
{
	public:
		anytime_astar(H* heuristic, E* expander, Q* queue) :
            heuristic_(heuristic), expander_(expander)
		{
			open_ = queue;
            cost_cutoff_ = warthog::COST_MAX;
            exp_cutoff_ = UINT32_MAX;
            time_cutoff_nanos_ = UINT64_MAX;
            on_relax_fn_ = 0;
            on_generate_fn_ = 0;
            on_expand_fn_ = 0;
            pi_.instance_id_ = UINT32_MAX;
		}

		virtual ~anytime_astar() { }

        virtual void
		get_pathcost(
                warthog::problem_instance& instance, warthog::solution& sol)
        {

            sol.reset();
            pi_ = instance;

			warthog::search_node* incumbent = search(sol);
			if(incumbent)
			{
                sol.sum_of_edge_costs_ = incumbent->get_g() + 
                    heuristic_->ub(incumbent->get_id(), pi_.target_id_);
            }
        }

        virtual void
		get_path(warthog::problem_instance& instance, warthog::solution& sol)
		{
            sol.reset();
            pi_ = instance;

			warthog::search_node* incumbent = search(sol);
			if(incumbent)
			{
                sol.sum_of_edge_costs_ = 
                    incumbent->get_g() + 
                    heuristic_->ub(
                            incumbent->get_id(), 
                            pi_.target_id_);

				// follow backpointers to extract the path from start -> incumbent
                warthog::search_node* current = incumbent;
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

            while(true)
            {
                uint32_t tmp_id = sol.path_.back();
                warthog::sn_id_t next_id = 
                    heuristic_->get_move(tmp_id, pi_.target_id_);
                if(next_id == warthog::SN_ID_MAX) 
                { break; }
                sol.path_.push_back(next_id);
            }
		}
        
        // return a list of the nodes expanded during the last search
        // @param coll: an empty list
        void
        closed_list(std::vector<warthog::search_node*>& coll)
        {
            for(size_t i = 0; i < expander_->get_nodes_pool_size(); i++)
            {
                warthog::search_node* current = expander_->generate(i);
                if(current->get_search_number() == pi_.instance_id_) 
                { 
                    coll.push_back(current);
                }
            }
        }

        // return a pointer to the warthog::search_node object associated
        // with node @param id. If this node was not generate during the 
        // last search instance, 0 is returned instead
        warthog::search_node*
        get_generated_node(warthog::sn_id_t id)
        {
            warthog::search_node* ret = expander_->generate(id);
            return ret->get_search_number() == pi_.instance_id_ ? ret : 0;
        }

        // apply @param fn to every node on the closed list
        void
        apply_to_closed(std::function<void(warthog::search_node*)>& fn)
        {
            for(size_t i = 0; i < expander_->get_nodes_pool_size(); i++)
            {
                warthog::search_node* current = expander_->generate(i);
                if(current->get_search_number() == pi_.instance_id_)
                { fn(current); }
            }
        }

        // apply @param fn every time a node is successfully relaxed
        void
        apply_on_relax(std::function<void(warthog::search_node*)>& fn)
        {
            on_relax_fn_ = &fn;
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

        // apply @param fn when a node is popped off the open list for 
        // expansion
        void
        apply_on_expand(std::function<void(warthog::search_node*)>& fn)
        {
            on_expand_fn_ = &fn;
        }

        // set a cost-cutoff to run a bounded-cost A* search.
        // the search terminates when the target is found or the f-cost 
        // limit is reached.
        inline void
        set_cost_cutoff(warthog::cost_t cutoff) { cost_cutoff_ = cutoff; }

        inline void
        set_time_cutoff(uint64_t nanos)
        {
            time_cutoff_nanos_ = nanos;
        }

        inline warthog::cost_t
        get_cost_cutoff() { return cost_cutoff_; }

        // set a cutoff on the maximum number of node expansions.
        // the search terminates when the target is found or when
        // the limit is reached
        inline void
        set_max_expansions_cutoff(uint32_t cutoff) { exp_cutoff_ = cutoff; }

        inline uint32_t 
        get_max_expansions_cutoff() { return exp_cutoff_; }  

		virtual inline size_t
		mem()
		{
			size_t bytes = 
				// memory for the priority quete
				open_->mem() + 
				// gridmap size and other stuff needed to expand nodes
				expander_->mem() +
                // heuristic uses some memory too
                heuristic_->mem() +
				// misc
				sizeof(*this);
			return bytes;
		}


	private:
		H* heuristic_;
		E* expander_;
		Q* open_;
        warthog::problem_instance pi_;

        // early termination limits
        warthog::cost_t cost_cutoff_; 
        uint32_t exp_cutoff_;
        uint64_t time_cutoff_nanos_;

        // callback for when a node is relaxed
        std::function<void(warthog::search_node*)>* on_relax_fn_;

        // callback for when a node is reached / generated
        std::function<void(
                warthog::search_node*, 
                warthog::search_node*, 
                warthog::cost_t edge_cost, 
                uint32_t edge_id)>* on_generate_fn_;

        // callback for when a node is expanded
        std::function<void(warthog::search_node*)>* on_expand_fn_;

		// no copy ctor
		anytime_astar(const anytime_astar& other) { } 
		anytime_astar& 
		operator=(const anytime_astar& other) { return *this; }

		warthog::search_node*
		search(warthog::solution& sol)
		{
			warthog::timer mytimer;
			mytimer.start();
			open_->clear();

			warthog::search_node* start;
            warthog::search_node* incumbent = 0;
            warthog::cost_t incumbent_lb = warthog::COST_MAX;
            warthog::cost_t incumbent_ub = warthog::COST_MAX;

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

            incumbent = start;
            incumbent_lb = heuristic_->h(pi_.start_id_, pi_.target_id_);
            incumbent_ub = heuristic_->ub(pi_.start_id_, pi_.target_id_);

			open_->push(start);
            sol.nodes_inserted_++;
            
            if(on_generate_fn_) 
            { (*on_generate_fn_)(start, 0, 0, UINT32_MAX); }



			#ifndef NDEBUG
			if(pi_.verbose_) { pi_.print(std::cerr); std:: cerr << "\n";}
			#endif

            // begin expanding
			while(open_->size())
			{
				warthog::search_node* current = open_->pop();

				#ifndef NDEBUG
				if(pi_.verbose_)
				{
					int32_t x, y;
                    expander_->get_xy(current->get_id(), x, y);
					std::cerr 
                        << sol.nodes_expanded_
                        << (current->get_expanded() ? ". re-expanding " : ". expanding ")
                        << "("<<x<<", "<<y<<")...";
					current->print(std::cerr);
					std::cerr << std::endl;
				}
				#endif

				current->set_expanded(true); // NB: set before generating
				assert(current->get_expanded());
				sol.nodes_expanded_++;
                if(on_expand_fn_) { (*on_expand_fn_)(current); }

                // evaluate the upper and lower bounds for the current node
                warthog::cost_t current_ub = 
                    current->get_g() +
                    heuristic_->ub(current->get_id(), pi_.target_id_);
                warthog::cost_t current_lb = 
                    current->get_g() + 
                    heuristic_->h(current->get_id(), pi_.target_id_);

                // terminate if we closed the gap
                if(current_ub == current_lb)
                {
                    incumbent = current;
                    incumbent_lb = current_lb;
                    incumbent_ub = current_ub;
                    break;
                }
                    
                // other termination criteria 
                if(current->get_f() > cost_cutoff_) { break; } 
                if(sol.nodes_expanded_ >= exp_cutoff_) { break; }
                if(mytimer.elapsed_time_nano() >= time_cutoff_nanos_) { break; }


                // generate successors
				expander_->expand(current, &pi_);
				warthog::search_node* n = 0;
				warthog::cost_t cost_to_n = 0;
                uint32_t edge_id = 0;
				for(uint32_t i = 0; i < expander_->get_num_successors(); i++)
				{
                    expander_->get_successor(i, n, cost_to_n);
                    sol.nodes_touched_++;
                    if(on_generate_fn_) 
                    { (*on_generate_fn_)(n, current, cost_to_n, edge_id++); }
                    
                    // add new nodes to the fringe
                    if(n->get_search_number() != current->get_search_number())
                    {
						warthog::cost_t gval = current->get_g() + cost_to_n;
                        warthog::cost_t hval = 
                            heuristic_->h(n->get_id(),pi_.target_id_);
                        warthog::cost_t ub_val = 
                            heuristic_->ub(n->get_id(),pi_.target_id_);

                        if((gval + ub_val) < incumbent_ub)
                        {
                            incumbent = n;
                            incumbent_lb = (gval + hval);
                            incumbent_ub = (gval + ub_val);
                        }

                        n->init(current->get_search_number(), current->get_id(),
                            gval, gval + hval);
                        open_->push(n);
                        sol.nodes_inserted_++;
                        if(on_relax_fn_) { (*on_relax_fn_)(n); }
                
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

                        continue;
                    }

                    // update an old node if we find a better way to reach it
                    // NB: we allow re-expansions
                    if((current->get_g() + cost_to_n) < n->get_g())
                    {
                        warthog::cost_t gval = current->get_g() + cost_to_n;
                        warthog::cost_t hval = 
                            heuristic_->h(n->get_id(),pi_.target_id_);
                        warthog::cost_t ub_val = 
                            heuristic_->h(n->get_id(),pi_.target_id_);

                        if((gval + ub_val) < incumbent_ub)

                        {
                            incumbent = n;
                            incumbent_lb = (gval + hval);
                            incumbent_ub = (gval + ub_val);
                        }

                        n->relax(gval, current->get_id());
                        if(open_->contains(n))
                        {
                            open_->decrease_key(n);
                        }
                        else
                        {
                            open_->push(n); 
                        }
                        sol.nodes_updated_++;
                        if(on_relax_fn_) { (*on_relax_fn_)(n); }

                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            int32_t x, y;
                            expander_->get_xy(n->get_id(), x, y);
                            std::cerr 
                                << "  open; updating (edgecost="
                                << cost_to_n<<") ("<<x<<", "<<y<<")...";
                            n->print(std::cerr);
                            std::cerr << std::endl;
                        }
                        #endif
                        continue;
                    }

                    // old nodes that don't need updating
                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        int32_t x, y;
                        expander_->get_xy(n->get_id(), x, y);
                        std::cerr 
                            << "  open; not updating (edgecost=" 
                            << cost_to_n<< ") ("<<x<<", "<<y<<")...";
                        n->print(std::cerr);
                        std::cerr << std::endl;
                    }
                    #endif
				}
			}

			mytimer.stop();
			sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
            sol.nodes_surplus_ = open_->size();

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                if(incumbent == 0) 
                {
                    std::cerr 
                        << "search failed; no solution exists " << std::endl;
                }
                else
                {
                    std::cerr 
                        << "target found. gap: " 
                        << (double)incumbent_ub / (double)incumbent_lb 
                        << std::endl;
                }
            }
            #endif

            return incumbent;
		}
};

}

#endif

