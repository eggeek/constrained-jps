//
// Run CPD extractions, it is assumed there exists a path from `start_id` to
// `target_id`, we have a few (debug) checks but that's all.
//
#ifndef __CPD_EXTRACTIONS_H_
#define __CPD_EXTRACTIONS_H_

#include "forward.h"
#include "graph_oracle.h"
#include "log.h"
#include "problem_instance.h"
#include "solution.h"
#include "timer.h"
#include <bits/stdint-uintn.h>
#include <climits>

namespace warthog
{

template<warthog::cpd::symbol T>
class cpd_extractions_base : public warthog::search
{
    public:
        cpd_extractions_base(warthog::graph::xy_graph *g,
                             warthog::cpd::graph_oracle_base<T> *oracle)
            : g_(g), oracle_(oracle)
        {
            assert(oracle->get_graph() == g);
            max_k_moves_ = UINT_MAX;
            time_cutoff_ = DBL_MAX;
        }

        virtual ~cpd_extractions_base() { }

        virtual void
        get_path(warthog::problem_instance& pi, warthog::solution& sol)
        {
            // NOTE We do not "hold" a pi_ in this class, is that a problem?
            sol.reset();

            warthog::timer mytimer;
            mytimer.start();

            warthog::sn_id_t source_id = pi.start_id_;
            warthog::sn_id_t target_id = pi.target_id_;

            // NB: we store the actual path in addition to simply extracting it
            sol.sum_of_edge_costs_ = 0;

            while(source_id != target_id && sol.nodes_touched_ < max_k_moves_)
            {
                sol.path_.push_back(source_id);

                uint32_t move = oracle_->get_move(source_id, target_id);
                if (move == warthog::cpd::CPD_FM_NONE)
                {
                    error("Could not find path", pi);
                    break;
                }

                warthog::graph::node* n = g_->get_node(source_id);
                assert(n->out_degree() > move);

                warthog::graph::edge* e = (n->outgoing_begin() + move);
                source_id = e->node_id_;
                sol.sum_of_edge_costs_ += e->wt_;
                sol.nodes_touched_++;
            }
            sol.path_.push_back(source_id);

            mytimer.stop();
            sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
        }

        virtual void
        get_pathcost(warthog::problem_instance& pi, warthog::solution& sol)
        {
            // NOTE We do not "hold" a pi_ in this class, is that a problem?
            sol.reset();

            warthog::timer mytimer;
            mytimer.start();

            warthog::sn_id_t source_id = pi.start_id_;
            warthog::sn_id_t target_id = pi.target_id_;

            sol.sum_of_edge_costs_ = 0;

            while(source_id != target_id && sol.nodes_touched_ < max_k_moves_)
            {
                uint32_t move = oracle_->get_move(source_id, target_id);
                if (move == warthog::cpd::CPD_FM_NONE)
                {
                    error("Could not find path", pi);
                    break;
                }

                warthog::graph::node* n = g_->get_node(source_id);
                assert(n->out_degree() > move);

                warthog::graph::edge* e = (n->outgoing_begin() + move);
                source_id = e->node_id_;
                sol.sum_of_edge_costs_ += e->wt_;
                sol.nodes_touched_++;
            }

            mytimer.stop();
            sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();
        }

        void
        set_max_k_moves(uint32_t k_moves)
        { max_k_moves_ = k_moves; }

        uint32_t
        get_max_k_moves()
        { return max_k_moves_; }

        virtual size_t
        mem()
        {
            return sizeof(*this);
        }

    private:
        warthog::graph::xy_graph* g_;
        warthog::cpd::graph_oracle_base<T>* oracle_;
        double time_cutoff_;            // Time limit in nanoseconds
        uint32_t max_k_moves_;          // Max "distance" from target
};

typedef cpd_extractions_base<warthog::cpd::FORWARD> cpd_extractions;

}

#endif // __CPD_EXTRACTIONS_H_
