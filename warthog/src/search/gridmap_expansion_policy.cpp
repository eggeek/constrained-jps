#include "gridmap_expansion_policy.h"
#include "helpers.h"
#include "problem_instance.h"
#include "global.h"

warthog::gridmap_expansion_policy::gridmap_expansion_policy(
		warthog::gridmap* map, bool manhattan)
: expansion_policy(map->height()*map->width()), map_(map), manhattan_(manhattan)
{
}

void 
warthog::gridmap_expansion_policy::expand(warthog::search_node* current,
		warthog::problem_instance* problem)
{
	reset();

	// get terrain type of each tile in the 3x3 square around (x, y)
	uint32_t tiles = 0;
	uint32_t nodeid = (uint32_t)current->get_id();
	map_->get_neighbours(nodeid, (uint8_t*)&tiles);

#ifdef CNT
  global::statis::update_gval(current);
#endif

//	#ifndef NDEBUG
//	uint32_t cx_, cy_;
//	warthog::helpers::index_to_xy(nodeid, map_->width(), cx_, cy_);
//	assert(tiles[0] == map_->get_label(cx_-1, cy_-1));
//	assert(tiles[1] == map_->get_label(cx_, cy_-1));
//	assert(tiles[2] == map_->get_label(cx_+1, cy_-1));
//	assert(tiles[3] == map_->get_label(cx_-1, cy_));
//	assert(tiles[4] == map_->get_label(cx_, cy_));
//	assert(tiles[5] == map_->get_label(cx_+1, cy_));
//	assert(tiles[6] == map_->get_label(cx_-1, cy_+1));
//	assert(tiles[7] == map_->get_label(cx_, cy_+1));
//	assert(tiles[8] == map_->get_label(cx_+1, cy_+1));
//	#endif

	// NB: no corner cutting or squeezing between obstacles!
	uint32_t nid_m_w = nodeid - map_->width();
	uint32_t nid_p_w = nodeid + map_->width();

	// generate cardinal moves
    if((tiles & 514) == 514) // N
	{  
		add_neighbour(this->generate(nid_m_w), 1);
	} 
	if((tiles & 1536) == 1536) // E
	{
		add_neighbour(this->generate(nodeid + 1), 1);
	}
	if((tiles & 131584) == 131584) // S
	{ 
		add_neighbour(this->generate(nid_p_w), 1);
	}
	if((tiles & 768) == 768) // W
	{ 
		add_neighbour(this->generate(nodeid - 1), 1);
	}
    if(manhattan_) { return; }

    // generate diagonal moves
	if((tiles & 1542) == 1542) // NE
	{ 
        add_neighbour(this->generate(nid_m_w + 1), warthog::DBL_ROOT_TWO);
	}
	if((tiles & 394752) == 394752) // SE
	{	
        add_neighbour(this->generate(nid_p_w + 1), warthog::DBL_ROOT_TWO);
	}
	if((tiles & 197376) == 197376) // SW
	{ 
        add_neighbour(this->generate(nid_p_w - 1), warthog::DBL_ROOT_TWO);
	}
	if((tiles & 771) == 771) // NW
	{ 
		add_neighbour(this->generate(nid_m_w - 1), warthog::DBL_ROOT_TWO);
	}


}

void
warthog::gridmap_expansion_policy::get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    
    map_->to_unpadded_xy((uint32_t)nid, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::gridmap_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

warthog::search_node*
warthog::gridmap_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if((uint32_t)pi->target_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return generate(padded_id);
}

size_t
warthog::gridmap_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(*this) + 
        map_->mem();
}
