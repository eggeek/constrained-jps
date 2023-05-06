#include "jps2_expansion_policy.h"
#include "global.h"
namespace G = global;

warthog::jps2_expansion_policy::jps2_expansion_policy(warthog::gridmap* map)
    : expansion_policy(map->height() * map->width())
{
	map_ = map;
	jpl_ = new warthog::jps::online_jump_point_locator2(map);
	jp_ids_.reserve(100);
}

warthog::jps2_expansion_policy::~jps2_expansion_policy()
{
	delete jpl_;
}

void 
warthog::jps2_expansion_policy::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();
    jp_ids_.clear();
    jp_costs_.clear();

#ifdef CNT
    G::statis::update_subopt_expd(current->get_id(), current->get_g());
    G::statis::update_pruneable(current);
#endif

	// compute the direction of travel used to reach the current node.
    // TODO: store this value with the jump point location so we don't need
    // to compute it all the time
    uint32_t p_id = current->get_parent();
    uint32_t c_id = current->get_id();
	warthog::jps::direction dir_c =
	   	//this->compute_direction((uint32_t)current->get_parent(), (uint32_t)current->get_id());
	   	this->compute_direction(p_id, c_id);

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
			jpl_->jump(d, current_id, goal_id, jp_ids_, jp_costs_);
		}
	}

	//uint32_t searchid = problem->get_searchid();
	for(uint32_t i = 0; i < jp_ids_.size(); i++)
	{
		// bits 0-23 store the id of the jump point
		// bits 24-31 store the direction to the parent
		uint32_t jp_id = jp_ids_.at(i);
    warthog::cost_t jp_cost = jp_costs_.at(i);
		warthog::search_node* mynode = generate(jp_id);
		add_neighbour(mynode, jp_cost);
#ifdef CNT
    G::statis::update_subopt_touch(mynode->get_id(), current->get_g()+jp_cost);
    G::statis::sanity_checking(mynode->get_id(), current->get_g()+jp_cost);
#endif
	}
}

//void
//warthog::jps2_expansion_policy::update_parent_direction(warthog::search_node* n)
//{
//    uint32_t jp_id = jp_ids_.at(this->get_current_successor_index());
//    assert(n->get_id() == (jp_id & warthog::jps::JPS_ID_MASK));
//    warthog::jps::direction pdir = 
//        (warthog::jps::direction)*(((uint8_t*)(&jp_id))+3);
//    n->set_pdir(pdir);
//}

void
warthog::jps2_expansion_policy::get_xy(warthog::sn_id_t sn_id, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)sn_id, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::jps2_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t start_id = (uint32_t)pi->start_id_;
    uint32_t max_id = map_->header_width() * map_->header_height();

    if(start_id >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(start_id);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

warthog::search_node*
warthog::jps2_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t target_id = (uint32_t)pi->target_id_;
    uint32_t max_id = map_->header_width() * map_->header_height();

    if(target_id  >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(target_id);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

warthog::jps::direction
warthog::jps2_expansion_policy::compute_direction(
        uint32_t n1_id, uint32_t n2_id)
{
    if(n1_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

    int32_t x, y, x2, y2;
    warthog::helpers::index_to_xy(n1_id, map_->width(), x, y);
    warthog::helpers::index_to_xy(n2_id, map_->width(), x2, y2);
    int32_t dx = abs(x2 - x);
    int32_t dy = abs(y2 - y);

    if(dx > dy)
    {
        if(x2 > x)
        { return warthog::jps::EAST; }

        return warthog::jps::WEST;
    }

    if(y2 > y) 
    { return warthog::jps::SOUTH; }

    return warthog::jps::NORTH;
}
