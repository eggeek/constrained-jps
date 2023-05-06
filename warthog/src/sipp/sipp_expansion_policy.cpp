#include "sipp/sipp_expansion_policy.h"

warthog::sipp_expansion_policy::sipp_expansion_policy(
    warthog::sipp_gridmap* sipp_map) : sipp_map_(sipp_map)
{
    sz_xy = sipp_map_->gm_->header_height() * sipp_map_->gm_->header_width();
    pool_.push_back(new warthog::mem::node_pool(sz_xy));
    neis_ = new warthog::arraylist<neighbour_record>(32);
}

warthog::sipp_expansion_policy::~sipp_expansion_policy()
{
    reset();
    delete neis_;
    for(uint32_t i = 0; i < pool_.size(); i++)
    {
        delete pool_.at(i);
    }
}
