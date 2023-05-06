#include "bch_expansion_policy.h"
#include "contraction.h"
#include "problem_instance.h"
#include "search_node.h"

warthog::bch_expansion_policy::bch_expansion_policy(
        warthog::graph::xy_graph* g, 
        bool backward)
    : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    backward_ = backward;

    if(backward_)
    {
        fn_begin_iter_ = &warthog::bch_expansion_policy::get_bwd_begin_iter;
        fn_end_iter_ = &warthog::bch_expansion_policy::get_bwd_end_iter;

        fn_rev_end_iter_ = &warthog::bch_expansion_policy::get_fwd_end_iter;
        fn_rev_begin_iter_ = &warthog::bch_expansion_policy::get_fwd_begin_iter;
    }
    else
    {
        fn_begin_iter_ = &warthog::bch_expansion_policy::get_fwd_begin_iter;
        fn_end_iter_ = &warthog::bch_expansion_policy::get_fwd_end_iter;

        fn_rev_begin_iter_ = &warthog::bch_expansion_policy::get_bwd_begin_iter;
        fn_rev_end_iter_ = &warthog::bch_expansion_policy::get_bwd_end_iter;
    }
}

void
warthog::bch_expansion_policy::expand(warthog::search_node* current,
        warthog::problem_instance* problem)
{
    reset();
    uint32_t current_id = (uint32_t)current->get_id();
    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter begin, end;

    // stall-on-demand
    begin = (this->*fn_rev_begin_iter_)(n);
    end = (this->*fn_rev_end_iter_)(n);
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        warthog::search_node* next = this->generate(e.node_id_);
        if(next->get_search_number() == current->get_search_number() &&
                current->get_g() > (next->get_g() + e.wt_))
        {
            return; // stall
        }
    }

    // OK, node doesn't need stalling; generate successors as usual
    begin = (this->*fn_begin_iter_)(n);
    end = (this->*fn_end_iter_)(n);
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        this->add_neighbour(this->generate(e.node_id_), e.wt_);
    }
}

size_t
warthog::bch_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(this);
}

warthog::search_node* 
warthog::bch_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = g_->to_graph_id((uint32_t)pi->start_id_);
    if(s_graph_id == warthog::INF32) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::bch_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id((uint32_t)pi->target_id_);
    if(t_graph_id == warthog::INF32) { return 0; }
    return generate(t_graph_id);
}
