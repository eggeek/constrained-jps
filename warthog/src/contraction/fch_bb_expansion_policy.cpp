#include "contraction.h"
#include "fch_bb_expansion_policy.h"
#include "xy_graph.h"
#include "search_node.h"

warthog::fch_bb_expansion_policy::fch_bb_expansion_policy(warthog::label::dfs_labelling* lab)
    : expansion_policy(lab->get_ch_data()->g_->get_num_nodes()), 
      chd_(lab->get_ch_data()), lab_(lab)
{
    t_label = s_label = INT32_MAX;
    filter = &warthog::fch_bb_expansion_policy::filter_bb_only;
}

void
warthog::fch_bb_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* instance)
{
    // TODO store one extra bit with each search node to indicate 
    // its parent-edge direction in the hierarchy (up or down)
    // the extra bit is used as an address offset for the 
    // appropriate successor generating function
    // this optimisation removes the need to compare the
    // level of the current node with the parent which 
    // currently costs three branching instructions and one 
    // read instruction with the possibility of a cache miss
    reset();

    uint32_t current_id = (uint32_t)current->get_id();
    uint32_t current_level = get_level(current_id);
    warthog::graph::node* n = chd_->g_->get_node(current_id);
    warthog::search_node* pn = generate(current->get_parent());

    // traveling up the hierarchy we generate all neighbours;
    // traveling down, we generate only "down" neighbours
    bool up_travel = !pn || (current_level > get_level((uint32_t)pn->get_id()));
    if(up_travel)
    {
        // generate outgoing up edges
        for(uint32_t i = 0; i < chd_->up_degree_->at(current_id); i++)
        {
            warthog::graph::edge& e = *(n->outgoing_begin() + i);
            assert(e.node_id_ < chd_->g_->get_num_nodes());
            if(!(this->*filter)(current_id, i))
            {
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
        // generate outgoing down edges
        for(uint32_t i = chd_->up_degree_->at(current_id);
                i < n->out_degree(); 
                i++)
        {
            warthog::graph::edge& e = *(n->outgoing_begin() + i);
            assert(e.node_id_ < chd_->g_->get_num_nodes());
            if(!(this->*filter)(current_id, i))
            {
                // prune down successors below the goal
                if(chd_->level_->at(e.node_id_) < t_level)
                { continue; }

                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
    }
    else
    {
        for(uint32_t i = chd_->up_degree_->at(current_id); 
                i < n->out_degree();
                i++)
        {
            warthog::graph::edge& e = *(n->outgoing_begin() + i);
            assert(e.node_id_ < chd_->g_->get_num_nodes());
            if(!(this->*filter)(current_id, i))
            {
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
    }
}

void
warthog::fch_bb_expansion_policy::get_xy(
        warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    chd_->g_->get_xy((uint32_t)nid, x, y);
}

warthog::search_node* 
warthog::fch_bb_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = chd_->g_->to_graph_id((uint32_t)pi->start_id_);
    if(s_graph_id == warthog::INF32) { return 0; }

    s_label = lab_->get_dfs_index(s_graph_id);

    return generate(s_graph_id);
}

warthog::search_node*
warthog::fch_bb_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    t_graph_id = chd_->g_->to_graph_id((uint32_t)pi->target_id_);
    if(t_graph_id == warthog::INF32) { return 0; }

    t_level = get_level(t_graph_id);
    t_label = lab_->get_dfs_index(t_graph_id);
    
    get_xy(t_graph_id, tx_, ty_);

    return generate(t_graph_id);
}
