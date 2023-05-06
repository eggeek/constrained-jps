#include "jps2_expansion_policy_prune2.h"
#include "constants.h"
#include "forward.h"
#include "global.h"
namespace G = global;

typedef warthog::jps2_expansion_policy_prune2 jps2_exp_prune2;

jps2_exp_prune2::jps2_expansion_policy_prune2(warthog::gridmap* map)
  : expansion_policy(map->height() * map->width())
{
	map_ = map;
	jpl_ = new warthog::online_jump_point_locator2_prune2(map, &jpruner);
  jpl_->init_tables();
	reset();
  costs_.clear();
  jp_ids_.clear();
	costs_.reserve(100);
	jp_ids_.reserve(100);
}

jps2_exp_prune2::~jps2_expansion_policy_prune2()
{
	delete jpl_;
}

void
warthog::jps2_expansion_policy_prune2::get_xy(warthog::sn_id_t sn_id, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)sn_id, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::jps2_expansion_policy_prune2::generate_start_node(
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
warthog::jps2_expansion_policy_prune2::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t target_id = (uint32_t)pi->target_id_;
    uint32_t max_id = map_->header_width() * map_->header_height();

    if(target_id  >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(target_id);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

void 
jps2_exp_prune2::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();
  jp_ids_.clear();
  costs_.clear();
  if (current->get_g() > 0 && current->get_parent() == NO_PARENT) {
    return;
  }
  jpruner.reset_constraints();
  jpl_->pa = current;

#ifdef CNT
  G::statis::update_subopt_expd(current->get_id(), current->get_g());
  G::statis::update_pruneable(current);
#endif
	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c = compute_direction(current->get_parent(), current->get_id());

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = problem->target_id_;

	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			jpl_->jump(d, current_id, goal_id, jp_ids_, costs_);
		}
	}

	for(uint32_t i = 0; i < jp_ids_.size(); i++)
	{
		// bits 0-23 store the id of the jump point
		// bits 24-31 store the direction to the parent
		uint32_t jp_id = jp_ids_.at(i);
		warthog::search_node* mynode = generate(jp_id);
    add_neighbour(mynode, costs_.at(i));

#ifdef CNT
    G::statis::update_subopt_touch(mynode->get_id(), current->get_g()+costs_.at(i));
    G::statis::sanity_checking(mynode->get_id(), current->get_g()+costs_.at(i));
#endif
	}
}
