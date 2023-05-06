#ifndef WARTHOG_BCH_SEARCH_H
#define WARTHOG_BCH_SEARCH_H

// bch_search.h
//
// Bidirectional Contration Hierarchies query algorithm.
// Similar to warthog::bidirectional_search but with some slight
// differences such as a different termination condition
//
// @author: dharabor
// @created: 2018-05-02
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

class expansion_policy;
class search_node;

template<class H, class E>
class bch_search : public warthog::search
{
    public:
        bch_search(E* fexp, E* bexp, H* heuristic) 
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

        ~bch_search()
        {
            delete fopen_;
            delete bopen_;
        }

        virtual void
        get_path(warthog::problem_instance& pi, warthog::solution& sol)
        {
            pi_ = pi;
            this->search(sol);
            if(best_cost_ != warthog::INF32) 
            { 
                sol.sum_of_edge_costs_ = best_cost_;
                reconstruct_path(sol);
            }

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "path: \n";
                for(size_t i = 0; i < sol.path_.size(); i++)
                {
                    std::cerr << sol.path_.at(i) << std::endl;
                }
            }
            #endif
        }

        virtual void
        get_pathcost(warthog::problem_instance& pi, warthog::solution& sol)
        {
            pi_ = pi;
            this->search(sol);
            if(sol.nodes_expanded_ > exp_cutoff_ ) { std::cerr << "wtf!"; exit(1); }
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

        //warthog::search_node* 
        //get_search_node(uint32_t id, int direction=0)
        //{
        //    warthog::search_node* ret = 0;
        //    if(direction == 0) { ret = fexpander_->generate(id); }
        //    else { ret = bexpander_->generate(id); }

        //    if(ret.get_search_node() == pi_.instance_id_)
        //    {
        //        return ret;
        //    }
        //    return 0;
        //}

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
        double best_cost_;
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
            while(true)
            {
               sol.path_.push_back(current->get_id());
               if(current->get_parent() == warthog::SN_ID_MAX) break;
               current = fexpander_->generate(current->get_parent());
            }
            std::reverse(sol.path_.begin(), sol.path_.end());

            current = w_;
            while(current->get_parent() != warthog::SN_ID_MAX)
            {  
               sol.path_.push_back(current->get_parent());
               current = bexpander_->generate(current->get_parent());
            }

        }

        // modify this function to balance the search
        // by default the search expands one node in
        // each direction then switches to the other direction
        bool
        forward_next()
        {
            warthog::cost_t fwd_min = warthog::INF32;
            warthog::cost_t bwd_min = warthog::INF32;
            bwd_min = bopen_->size() ? bopen_->peek()->get_f() : warthog::INF32;
            fwd_min = fopen_->size() ? fopen_->peek()->get_f() : warthog::INF32;
            return fwd_min < bwd_min;
        }

        void 
        search(warthog::solution& sol)
        {
            warthog::timer mytimer;
            mytimer.start();

            fopen_->clear();
            bopen_->clear();

            // init
            best_cost_ = warthog::INF32;
            v_ = w_ = 0;

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "bch_search. ";
                pi_.print(std::cerr);
                std::cerr << std::endl;
            }
            #endif

            // generate the start and target nodes.
            warthog::search_node *start, *target;
            start = fexpander_->generate_start_node(&pi_);
            target = bexpander_->generate_target_node(&pi_);
            start->init(pi_.instance_id_, warthog::SN_ID_MAX, 0, 
                        heuristic_->h(start->get_id(), target->get_id()));
            target->init(pi_.instance_id_, warthog::SN_ID_MAX, 0, 
                         heuristic_->h(start->get_id(), target->get_id()));
            fopen_->push(start);
            bopen_->push(target);

            // also update problem instance with internal ids (for debugging)
            pi_.start_id_ = start->get_id();
            pi_.target_id_ = target->get_id();

            // interleave search; we terminate when the best lower bound
            // is larger than the cost of the best solution so far (or when
            // we exhaust both open lists)
            //
            // NB: in a standard formulation we can terminate when either
            // open list is exhausted. however in a more general context
            // bi-directional search might choose to ignore an edge so it
            // can be explored in the opposite direction. thus one open list
            // can be exhausted and it still makes sense to continue the
            // search. such an algorithm might be more efficient
            // (e.g. CH with arc flags) 
            warthog::search_node *ftop = 0, *btop = 0;
            while(fopen_->size() || bopen_->size())
            {
                if(fopen_->size() > 0) { ftop = fopen_->peek(); } 
                if(bopen_->size() > 0) { btop = bopen_->peek(); } 

                // the code below computes the bound more simply and works
                // on directed planar graphs with asymmetric costs
                warthog::cost_t best_bound = std::min( 
                    ftop->get_f(), btop->get_f());

                if(best_bound > cost_cutoff_) { break; } 
                if(sol.nodes_expanded_ >= exp_cutoff_) { break; }

                // terminate if we cannot improve the best solution so far.
                // NB: bidirectional dijkstra stops when the two search 
                // frontiers expand the same node; bidirectional A* stops 
                // when the best bound is larger than the best solution
                if(best_bound > best_cost_)
//                        || (dijkstra_ && v_ && w_ && 
//                         v_->get_expanded() && w_->get_expanded()))
                {
#ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        std::cerr << "provably-best solution found; cost=" << 
                            best_cost_ << std::endl;
                    }
#endif
                    break;
                }

                // ok, we still have hope. let's keep expanding. 
                if(forward_next())
                {
                    if(ftop->get_f() < best_cost_ && fopen_->size()) 
                    {
                        warthog::search_node* current = fopen_->pop();
                        expand(current, fopen_, fexpander_, bexpander_, 
                                pi_.target_id_, sol);
                    }
                    else if(btop->get_f() < best_cost_ && bopen_->size())
                    {
                        // we can't improve the best solution in the forward
                        // direction so we ignore the interleave policy and 
                        // expand a node in the backward direction instead
                        warthog::search_node* current = bopen_->pop();
                        expand(current, bopen_, bexpander_, fexpander_, 
                                pi_.start_id_, sol);
                    }
                    else 
                    {
                        // search is finished in both directions
                        break;
                    }
                }
                else 
                {
                    if(btop->get_f() < best_cost_ && bopen_->size())
                    {
                        warthog::search_node* current = bopen_->pop();
                        expand(current, bopen_, bexpander_, fexpander_, 
                                pi_.start_id_, sol);
                    }
                    else if(ftop->get_f() < best_cost_ && fopen_->size())
                    {
                        // we can't improve the best solution in the backward
                        // direction so we ignore the interleave policy and 
                        // expand a node in the forward direction instead
                        warthog::search_node* current = fopen_->pop();
                        expand(current, fopen_, fexpander_, bexpander_, 
                                pi_.target_id_, sol);
                    }
                    else
                    {
                        // search is finished in both directions
                        break;
                    }
                }
            }

            assert(best_cost_ != warthog::INF32 || (v_ == 0 && w_ == 0));

			mytimer.stop();
			sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
            sol.nodes_surplus_ = fopen_->size() + bopen_->size();
        }

        void
        expand( warthog::search_node* current, warthog::pqueue_min* open, 
                E* expander, E* reverse_expander, 
                warthog::sn_id_t tmp_targetid, warthog::solution& sol)
        {
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

            // update the best solution if possible
            warthog::search_node* rev_current = 
                reverse_expander->generate(current->get_id());
            if(rev_current->get_search_number() == current->get_search_number())
            {
                if((current->get_g() + rev_current->get_g()) < best_cost_)
                {
                    v_ = current;
                    w_ = rev_current;
                    best_cost_ = current->get_g() + rev_current->get_g();

                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        int32_t x, y;
                        expander->get_xy(current->get_id(), x, y);
                        std::cerr <<"new best solution!  cost=" << best_cost_<<std::endl;
                    }
                    #endif
                }
            }
            
            // generate all neighbours
            warthog::search_node* n = 0;
            warthog::cost_t cost_to_n = warthog::INF32;
            for(expander->first(n, cost_to_n); 
                    n != 0; 
                    expander->next(n, cost_to_n))
            {
                sol.nodes_touched_++;

                // add new nodes to the fringe
                if(n->get_search_number() != current->get_search_number())
                {
                    warthog::cost_t gval = current->get_g() + cost_to_n;
                    n->init(current->get_search_number(), 
                            current->get_id(), 
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
                            n->relax(gval, current->get_parent());
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

