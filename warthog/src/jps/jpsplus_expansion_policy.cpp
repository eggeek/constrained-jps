#include "jpsplus_expansion_policy.h"

warthog::jpsplus_expansion_policy::jpsplus_expansion_policy(warthog::gridmap* map)
    : expansion_policy(map->width() * map->height())
{
	map_ = map;
	jpl_ = new warthog::offline_jump_point_locator(map);
}

warthog::jpsplus_expansion_policy::~jpsplus_expansion_policy()
{
	delete jpl_;
}

void 
warthog::jpsplus_expansion_policy::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c = this->compute_direction(
            (uint32_t)current->get_parent(), (uint32_t)current->get_id());

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = (uint32_t)current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = (uint32_t)problem->target_id_;
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			double jumpcost;
			uint32_t succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(succ_id != warthog::INF32)
			{
				add_neighbour(this->generate(succ_id), jumpcost);
			}
		}
	}
}

void
warthog::jpsplus_expansion_policy::get_xy(
        warthog::sn_id_t node_id, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)node_id, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::jpsplus_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

warthog::search_node*
warthog::jpsplus_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->target_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}
