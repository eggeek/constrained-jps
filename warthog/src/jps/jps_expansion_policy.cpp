#include "jps_expansion_policy.h"

warthog::jps_expansion_policy::jps_expansion_policy(warthog::gridmap* map)
    : expansion_policy(map->height()*map->width())
{
	map_ = map;
	jpl_ = new warthog::online_jump_point_locator(map);
	reset();
}

warthog::jps_expansion_policy::~jps_expansion_policy()
{
	delete jpl_;
}

void 
warthog::jps_expansion_policy::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c =
	   	this->compute_direction((uint32_t)current->get_parent(), (uint32_t)current->get_id());

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = (uint32_t)current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = (uint32_t)problem->target_id_;
    //uint32_t search_id = problem->get_searchid();
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
            warthog::cost_t jumpcost;
			uint32_t succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(succ_id != warthog::INF32)
			{
                warthog::search_node* jp_succ = this->generate(succ_id);
                //if(jp_succ->get_searchid() != search_id) { jp_succ->reset(search_id); }
                add_neighbour(jp_succ, jumpcost);
			}
		}
	}
}

void
warthog::jps_expansion_policy::get_xy(
        warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)nid, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::jps_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

warthog::search_node*
warthog::jps_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->target_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

inline warthog::jps::direction
warthog::jps_expansion_policy::compute_direction(
        uint32_t n1_id, uint32_t n2_id)
{
    if(n1_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

    int32_t x, y, x2, y2;
    warthog::helpers::index_to_xy(n1_id, map_->width(), x, y);
    warthog::helpers::index_to_xy(n2_id, map_->width(), x2, y2);
    warthog::jps::direction dir = warthog::jps::NONE;
    if(y2 == y)
    {
        if(x2 > x)
            dir = warthog::jps::EAST;
        else
            dir = warthog::jps::WEST;
    }
    else if(y2 < y)
    {
        if(x2 == x)
            dir = warthog::jps::NORTH;
        else if(x2 < x)
            dir = warthog::jps::NORTHWEST;
        else // x2 > x
            dir = warthog::jps::NORTHEAST;
    }
    else // y2 > y 
    {
        if(x2 == x)
            dir = warthog::jps::SOUTH;
        else if(x2 < x)
            dir = warthog::jps::SOUTHWEST;
        else // x2 > x
            dir = warthog::jps::SOUTHEAST;
    }
    assert(dir != warthog::jps::NONE);
    return dir;
}
