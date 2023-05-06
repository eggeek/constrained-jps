#include "apriori_filter.h"
#include "search_node.h"
#include "xy_graph.h"
#include <cassert>

warthog::apriori_filter::apriori_filter(warthog::graph::xy_graph* g) 
{
    g_ = g;
    filter_sz_ = g->get_num_nodes();
    filter_ = new uint8_t[filter_sz_];
    reset_filter();
}

warthog::apriori_filter::~apriori_filter()
{
    delete [] filter_;
}

void 
warthog::apriori_filter::set_flag_true(uint32_t node_id)
{
    filter_[node_id] = true;
}

void
warthog::apriori_filter::set_flag_false(uint32_t node_id)
{
    filter_[node_id] = false;
}

void 
warthog::apriori_filter::reset_filter()
{
    for(uint32_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = false;
    }
}

bool
warthog::apriori_filter::filter(uint32_t node_id, uint32_t edge_id)
{
    assert(node_id < g_->get_num_nodes());
    assert(edge_id < g_->get_node(node_id)->out_degree());
    return get_flag((g_->get_node(node_id)->outgoing_begin()+edge_id)->node_id_);
}

bool
warthog::apriori_filter::get_flag(uint32_t node_id) 
{
    return filter_[node_id];
}

