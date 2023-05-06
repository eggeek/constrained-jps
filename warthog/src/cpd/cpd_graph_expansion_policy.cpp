#include "cpd_graph_expansion_policy.h"


void 
warthog::cpd_graph_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* pi)
{
    reset();
    warthog::graph::xy_graph* g = oracle_->get_graph();
    warthog::graph::node* graph_node = g->get_node(current->get_id());

    uint32_t move_id = oracle_->get_move(current->get_id(), pi->target_id_);
    assert(move_id < graph_node->out_degree());

    warthog::graph::edge* fm = graph_node->outgoing_begin() + move_id;
    add_neighbour(generate(fm->node_id_), fm->wt_);
}

warthog::search_node* 
warthog::cpd_graph_expansion_policy::generate_start_node(warthog::problem_instance* pi)
{
    warthog::graph::xy_graph* g = oracle_->get_graph();
    uint32_t s_graph_id = g->to_graph_id((uint32_t)pi->start_id_);
    if(s_graph_id == warthog::INF32) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::cpd_graph_expansion_policy::generate_target_node(warthog::problem_instance* pi)
{
    // convert from external id to internal id
    warthog::graph::xy_graph* g = oracle_->get_graph();
    uint32_t t_graph_id = g->to_graph_id((uint32_t)pi->target_id_);
    if(t_graph_id == warthog::INF32) { return 0; }
    
    // generate the search node
    return generate(t_graph_id);
}

void
warthog::cpd_graph_expansion_policy::get_xy(sn_id_t node_id, int32_t& x, int32_t& y)
{
    oracle_->get_graph()->get_xy((uint32_t)node_id, x, y);
}
