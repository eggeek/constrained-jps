#ifndef WARTHOG_BIDIRECTIONAL_SEARCH_H
#define WARTHOG_BIDIRECTIONAL_SEARCH_H

// bidirectional_search.h
//
// A customisable variant of bidirectional best-first search.
// Users can pass in any heuristic and any (domain-specific) expansion policy.
//
// @author: dharabor
// @created: 2016-02-14
//

#include "constants.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"
#include "pqueue.h"
#include "search.h"
#include "search_node.h"
#include "solution.h"
#include "timer.h"
#include "zero_heuristic.h"

#include "constants.h"

#include <cstdlib>
#include <stack>
#include <cstdint>
#include <typeinfo>

namespace warthog
{

template<class H, class E>
class bidirectional_search : public warthog::search
{
    public:
        bidirectional_search(E* fexp, E* bexp, H* heuristic) 
            : fexpander_(fexp), bexpander_(bexp), heuristic_(heuristic)
        {
            fopen_ = new pqueue_min(512);
            bopen_ = new pqueue_min(512);
            
            dijkstra_ = false;
            if(typeid(*heuristic_) == typeid(warthog::zero_heuristic))
            {
                dijkstra_ = true;
            }

            exp_cutoff_ = warthog::INF32;
            cost_cutoff_ = warthog::INF32;
        }

        ~bidirectional_search()
        {
            delete fopen_;
            delete bopen_;
        }

        virtual void
        get_path(warthog::problem_instance& pi, warthog::solution& sol)
        {
            this->search(sol, pi);
            if(best_cost_ != warthog::INF32) 
            { 
                sol.sum_of_edge_costs_ = best_cost_;
                reconstruct_path(sol);
            }
        }

        virtual void
        get_pathcost(warthog::problem_instance& pi, warthog::solution& sol)
        {
            this->search(sol, pi);
            assert(sol.nodes_expanded_ <= exp_cutoff_);

            if(best_cost_ != warthog::INF32) 
            { sol.sum_of_edge_costs_ = best_cost_; }
        }
        
        // set a cost-cutoff to run a bounded-cost A* search.
        // the search terminates when the target is found or the f-cost 
        // limit is reached.
        inline void
        set_cost_cutoff(warthog::cost_t cutoff) { cost_cutoff_ = cutoff; }

        inline warthog::cost_t
        get_cost_cutoff() { return cost_cutoff_; }

        // set a cutoff on the maximum number of node expansions.
        // the search terminates when the target is found or when
        // the limit is reached
        inline void
        set_max_expansions_cutoff(uint32_t cutoff) { exp_cutoff_ = cutoff; }

        inline uint32_t 
        get_max_expansions_cutoff() { return exp_cutoff_; } 

        warthog::search_node* 
        get_search_node(uint32_t id, int direction=0)
        {
            if(direction == 0)
                return fexpander_->get_ptr(id, pi_.instance_id_);
            return bexpander_->get_ptr(id, pi_.instance_id_);
        }

        size_t
        mem()
        {
            return sizeof(*this) + 
                fopen_->mem() +
                bopen_->mem() +
                fexpander_->mem();
                bexpander_->mem();
        }

    private:
        warthog::pqueue_min* fopen_;
        warthog::pqueue_min* bopen_;
        E* fexpander_;
        E* bexpander_;
        H* heuristic_;
        bool dijkstra_;

        // early termination limits
        warthog::cost_t cost_cutoff_; 
        uint32_t exp_cutoff_;

        // v is the section of the path in the forward
        // direction and w is the section of the path
        // in the backward direction. need parent pointers
        // of both to extract the actual path
        warthog::search_node* v_;
        warthog::search_node* w_;
        warthog::cost_t best_cost_;
        warthog::problem_instance pi_;

        void
        reconstruct_path(warthog::solution& sol)
        {
            if(v_ && (&*v_ == &*bexpander_->generate(v_->get_id())))
            {
                warthog::search_node* tmp = v_;
                v_ = w_;
                w_ = tmp;
            }

            warthog::search_node* current = v_;
            while(current)
            {
               sol.path_.push_back(current->get_id());
               current = current->get_parent() == warthog::NO_PARENT ? 0 : 
                   fexpander_->generate(current->get_parent());
            }
            std::reverse(sol.path_.begin(), sol.path_.end());

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                for(auto& state : sol.path_)
                {
                    int32_t x, y;
                    fexpander_->get_xy(state, x, y);
                    std::cerr 
                        << "(f) final path: (" << x << ", " << y << ")...";
                    warthog::search_node* n = 
                        fexpander_->generate(state);
                    n->print(std::cerr);
                    std::cerr << std::endl;
                }
            }
            #endif

            // already added the meeting point node
            current = 0;
            if(w_ && w_->get_parent() != warthog::NO_PARENT)
            {
                current = bexpander_->generate(w_->get_parent());
            }

            while(current)
            {  
               #ifndef NDEBUG
                if(pi_.verbose_)
                {
                    int32_t x, y;
                    bexpander_->get_xy(current->get_id(), x, y);
                    std::cerr 
                        << "(b) final path: (" << x << ", " << y << ")...";
                    current->print(std::cerr);
                    std::cerr << std::endl;
                }
               #endif

               sol.path_.push_back(current->get_id());
               current = current->get_parent() == warthog::NO_PARENT ? 0 : 
                   bexpander_->generate(current->get_parent());
            }
        }

        // modify this function to balance the search
        // by default the search expands one node in
        // each direction then switches to the other direction
        bool
        forward_next()
        {
            warthog::cost_t fwd_min, bwd_min;
            bwd_min = bopen_->size() ? bopen_->peek()->get_f() : warthog::INF32;
            fwd_min = fopen_->size() ? fopen_->peek()->get_f() : warthog::INF32;
            return fwd_min < bwd_min;
        }

        void 
        search(warthog::solution& sol, warthog::problem_instance& pi_new)
        {
            warthog::timer mytimer;
            mytimer.start();

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "bidirectional_search. ";
                pi_new.print(std::cerr);
                std::cerr << std::endl;
            }
            #endif

            best_cost_ = warthog::COST_MAX;
            v_ = 0;
            w_ = 0;
            pi_ = pi_new;

            uint32_t fwd_instance_id;
            uint32_t bwd_instance_id;

            bool fwd_resume = 
                (bool)(pi_.extra_params_) &&
                fopen_->size() > 0 &&
                (fexpander_->generate_start_node(&pi_)->get_g() == 0) &&
                (fexpander_->generate_start_node(&pi_)->get_search_number() ==
                 fopen_->peek()->get_search_number());

            if(dijkstra_ && fwd_resume)
            { 
                // sometimes it is desirable to try and resume the 
                // previous forward search instead of starting fresh
                warthog::search_node* fwd_start = 
                    fexpander_->generate_start_node(&pi_);
                warthog::search_node* fwd_target = 
                    fexpander_->generate_target_node(&pi_);
                if( fwd_target->get_search_number() == 
                     fwd_start->get_search_number() )
                {
                    best_cost_ = fwd_target->get_g();
                    v_ = fwd_target;
                    w_ = 0;
                }
                fwd_instance_id = fwd_start->get_search_number();
            }
            else
            {
                // fresh forward search
                fwd_instance_id = pi_.instance_id_;
                warthog::search_node *start, *target;
                start = fexpander_->generate_start_node(&pi_);
                target = fexpander_->generate_target_node(&pi_);
                if(start == 0 ) { return; } // invalid start
                start->init(
                        fwd_instance_id,
                        warthog::NO_PARENT, 
                        0,
                        heuristic_->h(start->get_id(), target->get_id()));
                fopen_->clear();
                fopen_->push(start);
            }

            {
                // fresh backward search
                bwd_instance_id = pi_.instance_id_;
                warthog::search_node *bwd_start, *bwd_target;
                bwd_start = bexpander_->generate_target_node(&pi_);
                bwd_target = bexpander_->generate_start_node(&pi_);
                if(bwd_start == 0 ) { return; } // invalid start
                bwd_start->init(
                        bwd_instance_id,
                        warthog::NO_PARENT, 
                        0,
                        heuristic_->h(
                            bwd_start->get_id(), bwd_target->get_id()));
                bopen_->clear();
                bopen_->push(bwd_start);
            }

            // expand
            while(fopen_->size() || bopen_->size())
            {
                warthog::cost_t fwd_bound = fopen_->size() ? 
                    fopen_->peek()->get_f() : warthog::COST_MAX;
                warthog::cost_t bwd_bound = bopen_->size() ?
                    bopen_->peek()->get_f() : warthog::COST_MAX;

                warthog::cost_t best_bound = dijkstra_ ? 
                    (fwd_bound + bwd_bound) : std::min(fwd_bound, bwd_bound);

                if(best_bound >= best_cost_) { break; }
                if(best_bound > cost_cutoff_) { break; }
                if(sol.nodes_expanded_ >= exp_cutoff_) { break; }

                // always expand the most promising node in either direction
                if(forward_next())
                {
                    warthog::search_node* current = fopen_->pop();
                    expand(current, fopen_, fexpander_, bexpander_, 
                            pi_.target_id_, sol, 
                            v_, w_, 
                            fwd_instance_id, 
                            bwd_instance_id);
                }
                else 
                {
                    warthog::search_node* current = bopen_->pop();
                    expand(current, bopen_, bexpander_, fexpander_, 
                            pi_.start_id_, sol, 
                            w_, v_, 
                            bwd_instance_id, 
                            fwd_instance_id);
                }
            }

            if(best_cost_ > cost_cutoff_)
            {
                v_ = 0;
                w_ = 0;
                best_cost_ = warthog::COST_MAX;
            }

			mytimer.stop();
			sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
            sol.nodes_surplus_ = fopen_->size() + bopen_->size();
        }

        void
        expand( warthog::search_node* current, 
                warthog::pqueue_min* open, E* expander, E* reverse_expander, 
                warthog::sn_id_t tmp_targetid, warthog::solution& sol,
                warthog::search_node*& fwd_meet, warthog::search_node*& bwd_meet,
                uint32_t fwd_instance_id, uint32_t bwd_instance_id)
        {
            if(current == 0) { return; }
            current->set_expanded(true);
            expander->expand(current, &pi_);
            sol.nodes_expanded_++;

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr 
                    << sol.nodes_expanded_ 
                    << ". expanding " 
                    << (pi_.target_id_ == tmp_targetid ? "(f)" : "(b)")
                    << " ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
            }
            #endif
            
            // generate all neighbours
            warthog::search_node* n = 0;
            warthog::cost_t cost_to_n = warthog::COST_MAX;
            for(expander->first(n, cost_to_n); 
                    n != 0; 
                    expander->next(n, cost_to_n))
            {
                sol.nodes_touched_++;

                // add new nodes to the fringe
                if(n->get_search_number() != current->get_search_number())
                {
                    warthog::cost_t gval = current->get_g() + cost_to_n;
                    n->init(current->get_search_number(), current->get_id(), 
                            gval,
                            gval + heuristic_->h(n->get_id(), tmp_targetid));
                    open->push(n);
                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        int32_t x, y;
                        expander->get_xy(n->get_id(), x, y);
                        std::cerr << "  generating "
                            << "(edgecost=" << cost_to_n<<") " 
                            << "("<<x<<", "<<y<<")...";
                        n->print(std::cerr);
                        std::cerr << std::endl;
                    }
                    #endif
                    sol.nodes_inserted_++;
                }

                // process neighbour nodes that the search has seen before
                else
                {
                    // skip neighbours already expanded
                    if(n->get_expanded())
                    {
                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            int32_t x, y;
                            expander->get_xy(n->get_id(), x, y);
                            std::cerr << "  closed; (edgecost=" << cost_to_n << ") "
                                << "("<<x<<", "<<y<<")...";
                            n->print(std::cerr);
                            std::cerr << std::endl;

                        }
                        #endif
                        continue;
                    }

                    // relax nodes on the fringe
                    if(open->contains(n))
                    {
                        warthog::cost_t gval = current->get_g() + cost_to_n;
                        if(gval < n->get_g())
                        {
                            n->relax(gval, current->get_id());
                            open->decrease_key(n);
                            sol.nodes_updated_++;
                            #ifndef NDEBUG
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << "  open; updating "
                                    << "(edgecost="<< cost_to_n<<") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                            }
                            #endif
                        }
                        #ifndef NDEBUG
                        else
                        {
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << "  open; not updating "
                                    << "(edgecost=" << cost_to_n<< ") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                            }
                        }
                        #endif
                    }
                }

                // update the best solution if possible
                warthog::search_node* rev_n = 
                    reverse_expander->generate(n->get_id());
                if(rev_n->get_search_number() == bwd_instance_id)
                {
                    if((n->get_g() + rev_n->get_g()) < best_cost_)
                    {
                        fwd_meet = n;
                        bwd_meet = rev_n;
                        best_cost_ = n->get_g() + rev_n->get_g();

                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            int32_t x, y;
                            expander->get_xy(n->get_id(), x, y);
                            std::cerr <<"new best solution!  cost=" << best_cost_<<std::endl;
                        }
                        #endif
                    }
                }
            }


            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr <<"closing ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
            }
            #endif
        }

        // clear the open lists and return all memory allocated for nodes
        // to the node pool
        void
        reclaim()
        {
            fopen_->clear();
            bopen_->clear();
            fexpander_->reclaim();
            bexpander_->reclaim();
        }
};

}

#endif

