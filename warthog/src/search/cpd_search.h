#ifndef CPD_SEARCH_H
#define CPD_SEARCH_H

// cpd_search.h
//
// A* implementation using an upper bound (CPD really) to enable:
//  - bounded sub-optimal search;
//  - heuristic weighting;
//  - anytime search;
//  - k-move search.
//
// @author: amaheo
// @created: 26/02/20
//

#include "constants.h"
#include "cpool.h"
#include "pqueue.h"
#include "problem_instance.h"
#include "search.h"
#include "solution.h"
#include "timer.h"
#include "vec_io.h"
#include "log.h"
#include "dummy_listener.h"

#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace warthog
{

// H is a heuristic function
// E is an expansion policy
// Q is the open list
// L is a "listener" which is used for callbacks
template< class H,
          class E,
          class Q = warthog::pqueue_min,
          class L = warthog::dummy_listener >
class cpd_search : public warthog::search
{
  public:
    cpd_search(H* heuristic, E* expander, Q* queue, L* listener = 0) :
        heuristic_(heuristic), expander_(expander), open_(queue),
        listener_(listener)
    {
        cost_cutoff_ = DBL_MAX;
        exp_cutoff_ = UINT32_MAX;
        time_cutoff_ = DBL_MAX;
        max_k_moves_ = UINT32_MAX;
        pi_.instance_id_ = UINT32_MAX;
        quality_cutoff_ = 0.0;
        k_moves_ =
            std::vector<uint32_t>(expander_->get_g()->get_num_nodes(), 0);
    }

    virtual ~cpd_search() { }

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
            assert(sol.path_.back() == pi_.start_id_);

            DO_ON_DEBUG_IF(pi_.verbose_)
            {
                for(auto& node_id : sol.path_)
                {
                    int32_t x, y;
                    expander_->get_xy(node_id, x, y);
                    std::cerr
                            << "final path: (" << x << ", " << y << ")...";
                    warthog::search_node* n =
                            expander_->generate(node_id);
                    assert(n->get_search_number() == pi_.instance_id_);
                    n->print(std::cerr);
                    std::cerr << std::endl;
                }
            }
        }
        std::reverse(sol.path_.begin(), sol.path_.end());
    }

    // return a list of the nodes expanded during the last search
    // @param coll: an empty list
    void
    closed_list(std::vector<warthog::search_node*>& coll)
    {
        for(size_t i = 0; i < expander_->get_node_pool_size(); i++)
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

    // Set a time limit cutoff
    inline void
    set_max_time_cutoff(double cutoff) { time_cutoff_ = cutoff; }

    inline void
    set_max_us_cutoff(double cutoff) { set_max_time_cutoff(cutoff * 1e3); }

    inline void
    set_max_ms_cutoff(double cutoff) { set_max_time_cutoff(cutoff * 1e6); }

    inline void
    set_max_s_cutoff(double cutoff) { set_max_time_cutoff(cutoff * 1e9); }

    inline double
    get_max_time_cutoff() { return time_cutoff_; }

    // Set a k-radius cut-off -- stop expanding nodes further than k moves away
    // from the start.
    inline void
    set_max_k_moves(uint32_t k_moves) { max_k_moves_ = k_moves; }

    inline uint32_t
    get_max_k_moves() { return max_k_moves_; }

    // Set a quality cut-off, if the LB is within xx% of the UB we can stop
    inline void
    set_quality_cutoff(double cutoff) { quality_cutoff_ = cutoff; }

    inline double
    get_quality_cutoff() { return quality_cutoff_; }

    void
    set_listener(L* listener)
    { listener_ = listener; }

    E*
    get_expander()
    { return expander_; }

    H*
    get_heuristic()
    { return heuristic_; }

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
    L* listener_;
    warthog::problem_instance pi_;

    // early termination limits
    warthog::cost_t cost_cutoff_;   // Fixed upper bound
    uint32_t exp_cutoff_;           // Number of iterations
    double time_cutoff_;            // Time limit in nanoseconds
    uint32_t max_k_moves_;          // Max "distance" from target
    std::vector<uint32_t> k_moves_; // "Distance" from target
    double quality_cutoff_;

    // no copy ctor
    cpd_search(const cpd_search& other) { }
    cpd_search&
    operator=(const cpd_search& other) { return *this; }

    /**
     * Determine whether we should be pruning a node or adding it to the open
     * list.
     */
    bool
    should_prune_(warthog::search_node *incumbent,
                  warthog::search_node *n)
    {
        bool prune = false;

        // if not [incumbent = nil or f(n) < f(incumbent)]
        if (incumbent != nullptr)
        {
            warthog::cost_t bound;

            // If we have an UB, we need to use it as source-of-truth
            if (incumbent->get_ub() < warthog::COST_MAX)
            {
                bound = incumbent->get_ub();
            }
            else
            {
                bound = incumbent->get_f();
            }

            if (n->get_f() >= bound)
            {
                debug(pi_.verbose_, "Prune by f-val:", n->get_f());
                prune = true;
            }
            // TODO Do we need to make a case where we have an incumbent's UB
            // but the current node does not?
        }

        return prune;
    }

    bool
    early_stop_(warthog::search_node* current,
                warthog::search_node* incumbent,
                warthog::solution* sol,
                warthog::timer* mytimer)
    {
        bool stop = false;

        mytimer->stop();
        // early termination: in case we want bounded-cost
        // search or if we want to impose some memory limit
        if(current->get_f() > cost_cutoff_)
        {
            info(pi_.verbose_, "Cost cutoff", current->get_f(), ">",
                  cost_cutoff_);
            stop = true;
        }
        if(sol->nodes_expanded_ >= exp_cutoff_)
        {
            info(pi_.verbose_, "Expanded cutoff", sol->nodes_expanded_, ">",
                  exp_cutoff_);
            stop = true;
        }
        // Exceeded time limit
        if (mytimer->elapsed_time_nano() > time_cutoff_)
        {
            info(pi_.verbose_, "Time cutoff", mytimer->elapsed_time_nano(),
                  ">", time_cutoff_);
            stop = true;
        }
        // Extra early-stopping criteria when we have an upper bound; in
        // CPD terms, we have an "unperturbed path."
        if (current->get_f() == current->get_ub())
        {
            info(pi_.verbose_, "Early stop");
            stop = true;
        }

        if (incumbent != nullptr && incumbent->get_ub() < warthog::COST_MAX)
        {
            if (current->get_f() * (1 + quality_cutoff_) > incumbent->get_ub())
            {
                info(pi_.verbose_, "Quality cutoff", current->get_f(), "x",
                      quality_cutoff_ + 1, ">", incumbent->get_ub());
                stop = true;
            }
        }

        if (k_moves_.at(current->get_id()) >= max_k_moves_)
        {
            info(pi_.verbose_,
                 "Reached maximum radius", k_moves_.at(current->get_id()));
            stop = true;
        }

        // A bit of a travestite use here.
        return stop;
    }

    /**
     * Udate the distance from the start to @n_id@ coming from @p_id@.
     */
    void
    update_k_(warthog::sn_id_t n_id, warthog::sn_id_t p_id)
    {
        k_moves_.at(n_id) = k_moves_.at(p_id) + 1;

        debug(pi_.verbose_, "Node", n_id, "set to k=", k_moves_.at(n_id));
    }

    /**
     * Initialise a new 'search_node' for the ongoing search given the parent
     * node (@param current).
     */
    void
    generate_node_(warthog::search_node* current,
                   warthog::search_node* n,
                   warthog::cost_t gval)
    {
        warthog::cost_t hval;
        warthog::cost_t ub;

        heuristic_->h(n->get_id(), pi_.target_id_, hval, ub);

        // Only update the UB if we have one
        if (ub < warthog::COST_MAX) { ub += gval; }

        // Should we check for overflow here?
        n->init(current->get_search_number(), current->get_id(),
                gval, gval + hval, ub);

        update_k_(n->get_id(), current->get_id());
    }

    void
    update_incumbent_(warthog::search_node* &incumbent, warthog::search_node* n)
    {
        // if n_i is a goal node
        if(expander_->is_target(n, &pi_))
        {
            incumbent = n;
            incumbent->set_ub(n->get_g());
            debug(pi_.verbose_,
                  "New path to target:", n->get_id(), "=", n->get_g());
        }
        else if (n->get_ub() < warthog::COST_MAX)
        {
            // Found a new incumbent
            if (incumbent == nullptr)
            {
                debug(pi_.verbose_, "Found UB:", n->get_id(), "=", n->get_ub());
                incumbent = n;
            }
            // Better incumbent
            else if (n->get_ub() < incumbent->get_ub())
            {
                debug(pi_.verbose_, "Update UB:", n->get_id(), "=", n->get_ub());
                incumbent = n;
            }
        }
    }

    // TODO refactor node information inside Stats
    warthog::search_node*
    search(warthog::solution& sol)
    {
        warthog::timer mytimer;
        mytimer.start();
        open_->clear();

        warthog::search_node* start;
        warthog::search_node* incumbent = nullptr;

        // get the internal target id
        if(pi_.target_id_ != warthog::SN_ID_MAX)
        {
            warthog::search_node* target =
                    expander_->generate_target_node(&pi_);
            if(!target) { return nullptr; } // invalid target location
            pi_.target_id_ = target->get_id();
        }

        // initialise and push the start node
        if(pi_.start_id_ == warthog::SN_ID_MAX) { return nullptr; }
        start = expander_->generate_start_node(&pi_);
        if(!start) { return nullptr; } // invalid start location
        pi_.start_id_ = start->get_id();

        // This heuristic also returns its upper bound
        warthog::cost_t start_h;
        warthog::cost_t start_ub;
        heuristic_->h(pi_.start_id_, pi_.target_id_, start_h, start_ub);

        // `hscale` is contained in the heuristic
        start->init(pi_.instance_id_, warthog::SN_ID_MAX, 0, start_h, start_ub);

        // Start node has no parent
        k_moves_.at(start->get_id()) = 0;
        listener_->generate_node(0, start, 0, UINT32_MAX);

        user(pi_.verbose_, pi_);
        info(pi_.verbose_, "cut-off =", cost_cutoff_, "- tlim =", time_cutoff_,
             "- k-move =", max_k_moves_, "- quality =", quality_cutoff_);
        debug(pi_.verbose_, "Start node:", *start);

        mytimer.stop();
        if (mytimer.elapsed_time_nano() > time_cutoff_)
        {
            // Bail without an answer if we exceed the time limit after the
            // first path extraction.
            info(pi_.verbose_, "Time cutoff before start of search.");
        }
        else
        {
            open_->push(start);
            sol.nodes_inserted_++;
        }

        // begin expanding
        while(open_->size())
        {
            warthog::search_node* current = open_->pop();
            update_incumbent_(incumbent, current);

            if (early_stop_(current, incumbent, &sol, &mytimer)) { break; }

            current->set_expanded(true); // NB: set before generating
            assert(current->get_expanded());
            sol.nodes_expanded_++;

            expander_->expand(current, &pi_);
            listener_->expand_node(current);

            // Incorrect timings reported otherwise
            DO_ON_DEBUG
            {
                mytimer.stop();

                info(pi_.verbose_, "[", mytimer.elapsed_time_micro(),"]",
                    sol.nodes_expanded_, "- Expanding:", *current);
            }

            // Generate successors of the current node
            warthog::cost_t cost_to_n = 0;
            uint32_t edge_id = 0;
            warthog::search_node* n;
            for(expander_->first(n, cost_to_n);
                n != nullptr;
                expander_->next(n, cost_to_n))
            {
                warthog::cost_t gval = current->get_g() + cost_to_n;

                sol.nodes_touched_++;
                edge_id++;
                listener_->generate_node(current, n, gval, edge_id);

                // Generate new search nodes
                if(n->get_search_number() != current->get_search_number())
                {
                    generate_node_(current, n, gval);
                }
                // if n_i \in OPEN u CLOSED and g(n_i) > g(n) + c(n, n_i)
                else if (gval < n->get_g())
                {
                    listener_->relax_node(n);

                    n->relax(gval, current->get_id());
                    update_k_(n->get_id(), current->get_id());
                    sol.nodes_updated_++;
                }
                // Neither a new node nor an improving path to it
                else
                {
                    trace(pi_.verbose_, "Closed;", *n);
                    continue;
                }

                // Beware, we need to prune *after updating* otherwise we may
                // have a cache miss of some sort: a node gets generated, then
                // pruned, then touched and pruned before being updated.
                if (should_prune_(incumbent, n))
                {
                    trace(pi_.verbose_, "Pruning:", *n);
                    continue;
                }

                // g(n_i) <- g(n) + c(n, n_i)
                if(open_->contains(n))
                {
                    open_->decrease_key(n);
                    trace(pi_.verbose_, "Updating:", *n);
                }
                // if n_i \in CLOSED
                else
                {
                    open_->push(n);
                    trace(pi_.verbose_, "Generating:", *n);
                    sol.nodes_inserted_++;
                }
            }
        }

        mytimer.stop();
        sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
        sol.nodes_surplus_ = open_->size();

        DO_ON_DEBUG_IF(pi_.verbose_)
        {
            if(incumbent == nullptr)
            {
                warning(pi_.verbose_, "Search failed; no solution exists.");
            }
            else
            {
                user(pi_.verbose_, "Best incumbent", *incumbent);
            }
        }

        // In case the incumbent is not the target, this means we found a better
        // path by *following the heuristic*, so we rebuild it in the same way.
        while (incumbent != nullptr && !expander_->is_target(incumbent, &pi_))
        {
            warthog::sn_id_t p_id = incumbent->get_id();
            warthog::sn_id_t n_id = heuristic_->get_move(p_id, pi_.target_id_);

            if (n_id == warthog::SN_ID_MAX)
            {
                // warning(pi_.verbose_, "Cannot rebuild path from", p_id);
                std::cerr << "Cannot rebuild path from " << p_id << std::endl;
                incumbent = nullptr;
            }
            else
            {
                warthog::search_node* n = expander_->generate(n_id);
                warthog::cost_t lb;
                warthog::cost_t ub;
                warthog::cost_t gval;

                heuristic_->h(n_id, pi_.target_id_, lb, ub);
                // The g-value of the next node is the g-value of the current
                // one plus the difference in their heuristic costs.
                //
                // We assume that either all nodes have UBs or none do.
                if (ub < warthog::COST_MAX)
                {
                    gval = incumbent->get_ub() - ub;
                }
                else
                {
                    gval = incumbent->get_f() - lb;
                }

                n->init(incumbent->get_search_number(),
                        p_id, gval, gval + lb, gval + ub);
                debug(pi_.verbose_, "Rebuild", *n);
                incumbent = n;
            }
        }

        return incumbent;
    }
};

}

#endif
