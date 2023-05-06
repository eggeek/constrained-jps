#include "sipp/temporal_jps_expansion_policy.h"

warthog::temporal_jps_expansion_policy::temporal_jps_expansion_policy(
    warthog::jpst_gridmap* jpst_gm) : jpst_gm_(jpst_gm)
{
    gm_map_width_ = jpst_gm_->gm_->width();
    map_width_ = jpst_gm_->gm_->header_width();
    sz_xy = jpst_gm_->gm_->header_height() * jpst_gm_->gm_->header_width();

    pool_.push_back(new warthog::mem::node_pool(sz_xy));
    neis_ = new warthog::arraylist<neighbour_record>(32);
    jpl_t_ = new warthog::jpst_locator(jpst_gm_);

    assert(warthog::cbs::NORTH == 0);
    assert(warthog::cbs::SOUTH == 1);
    assert(warthog::cbs::EAST == 2);
    assert(warthog::cbs::WEST == 3);

    // incompatible moves that can lead to 
    // edge collisions
    ec_moves_[warthog::cbs::NORTH] = warthog::cbs::SOUTH;
    ec_moves_[warthog::cbs::SOUTH] = warthog::cbs::NORTH;
    ec_moves_[warthog::cbs::EAST] = warthog::cbs::WEST;
    ec_moves_[warthog::cbs::WEST] = warthog::cbs::EAST;

    // for quickly computing successor xy_id values
    xy_id_offsets_[warthog::cbs::NORTH] = (uint32_t)(-1 * (int32_t)map_width_);
    xy_id_offsets_[warthog::cbs::SOUTH] = map_width_;
    xy_id_offsets_[warthog::cbs::EAST] = 1;
    xy_id_offsets_[warthog::cbs::WEST] = (uint32_t)-1;

    // for quickly computing successor gm_id values
    gm_id_offsets_[warthog::cbs::NORTH] = (uint32_t)(-1 * (int32_t)gm_map_width_);
    gm_id_offsets_[warthog::cbs::SOUTH] = gm_map_width_;
    gm_id_offsets_[warthog::cbs::EAST] = 1;
    gm_id_offsets_[warthog::cbs::WEST] = (uint32_t)-1;

    opposite_dir_[warthog::cbs::NORTH] = warthog::jps::SOUTH;
    opposite_dir_[warthog::cbs::SOUTH] = warthog::jps::NORTH;
    opposite_dir_[warthog::cbs::EAST] = warthog::jps::WEST;
    opposite_dir_[warthog::cbs::WEST] = warthog::jps::EAST;
}

warthog::temporal_jps_expansion_policy::~temporal_jps_expansion_policy()
{
    reset();

    delete jpl_t_;
    delete neis_;

    for(uint32_t i = 0; i < pool_.size(); i++)
    {
        delete pool_.at(i);
    }
}
