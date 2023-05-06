#include "expansion_policy.h"

warthog::expansion_policy::expansion_policy(size_t nodes_pool_size)
{
    nodes_pool_size_ = nodes_pool_size;
    nodepool_ = new warthog::mem::node_pool(nodes_pool_size);
    //neis_ = new std::vector<neighbour_record>();
    //neis_->reserve(32);
    neis_ = new warthog::arraylist<neighbour_record>(32);
}

warthog::expansion_policy::~expansion_policy()
{
    reset();
    delete neis_;
    delete nodepool_;
}
