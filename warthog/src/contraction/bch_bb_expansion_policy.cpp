#include "bb_filter.h"
#include "bch_bb_expansion_policy.h"
#include "contraction.h"
#include "problem_instance.h"
#include "search_node.h"

warthog::bch_bb_expansion_policy::bch_bb_expansion_policy(
        warthog::label::dfs_labelling* lab, bool backward)
    : expansion_policy(lab->get_ch_data()->g_->get_num_nodes()), 
      lab_(lab), backward_(backward)
{
    if(backward_)
    {
        fn_begin_iter_ = &warthog::bch_bb_expansion_policy::get_bwd_begin_iter;
        fn_end_iter_ = &warthog::bch_bb_expansion_policy::get_bwd_end_iter;

        fn_rev_end_iter_ = &warthog::bch_bb_expansion_policy::get_fwd_end_iter;
        fn_rev_begin_iter_ = &warthog::bch_bb_expansion_policy::get_fwd_begin_iter;

        fn_bb_filter_ = &warthog::bch_bb_expansion_policy::filter_bb_bwd;
    }
    else
    {
        fn_begin_iter_ = &warthog::bch_bb_expansion_policy::get_fwd_begin_iter;
        fn_end_iter_ = &warthog::bch_bb_expansion_policy::get_fwd_end_iter;

        fn_rev_begin_iter_ = &warthog::bch_bb_expansion_policy::get_bwd_begin_iter;
        fn_rev_end_iter_ = &warthog::bch_bb_expansion_policy::get_bwd_end_iter;

        fn_bb_filter_ = &warthog::bch_bb_expansion_policy::filter_bb_fwd;
    }
}

void
warthog::bch_bb_expansion_policy::expand(warthog::search_node* current,
        warthog::problem_instance* problem)
{
    reset();

    uint32_t current_id = (uint32_t)current->get_id();
    warthog::graph::node* n = lab_->get_ch_data()->g_->get_node(current_id);
   
    warthog::graph::edge_iter begin, end;

    // stall-on-demand
    // Pruning technique that avoids generating/expanding 
    // nodes whose g-value can be improved by the search
    // coming from the opposite direction.
    begin = (this->*fn_rev_begin_iter_)(n);
    end = (this->*fn_rev_end_iter_)(n);
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < lab_->get_ch_data()->g_->get_num_nodes());
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
        assert(e.node_id_ < lab_->get_ch_data()->g_->get_num_nodes());
        if(!(this->*fn_bb_filter_)(current_id, (uint32_t)(it - begin)))
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

size_t
warthog::bch_bb_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(this);
}

void
warthog::bch_bb_expansion_policy::get_xy(
        warthog::sn_id_t node_id, int32_t& x, int32_t& y)
{
    lab_->get_ch_data()->g_->get_xy((uint32_t)node_id, x, y);
}

warthog::search_node* 
warthog::bch_bb_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    // update the filter with the new target location
    uint32_t t_graph_id = 
        lab_->get_ch_data()->g_->to_graph_id((uint32_t)pi->target_id_);
    if(t_graph_id != warthog::INF32) 
    { 
        lab_->get_ch_data()->g_->get_xy(t_graph_id, tx_, ty_);
    }

    // generate the start node
    uint32_t s_graph_id = 
        lab_->get_ch_data()->g_->to_graph_id((uint32_t)pi->start_id_);
    if(s_graph_id == warthog::INF32) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::bch_bb_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = 
        lab_->get_ch_data()->g_->to_graph_id((uint32_t)pi->target_id_);
    if(t_graph_id == warthog::INF32) { return 0; }

    // update the filter with the new target location
    {
        lab_->get_ch_data()->g_->get_xy(t_graph_id, tx_, ty_);
    }
    return generate(t_graph_id);
}
