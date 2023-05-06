#include "cbs.h"
#include "cbs_ll_heuristic.h"
#include "cbs_ll_expansion_policy.h"
#include "grid.h"
#include "helpers.h"
#include "problem_instance.h"

#include <algorithm>

using namespace warthog::cbs;

warthog::cbs_ll_expansion_policy::cbs_ll_expansion_policy(
		warthog::gridmap* map, warthog::cbs_ll_heuristic* h) 
    : map_(map), h_(h)
{
    neis_ = new warthog::arraylist<neighbour_record>(32);

    map_xy_sz_ = map->height() * map->width();
    assert(map_xy_sz_ > 0);

    cons_ = new warthog::mapf::time_constraints<warthog::cbs::cbs_constraint>
                (map_xy_sz_);

    // preallocate memory for up to some number of timesteps 
    // in advance. for subsequent timesteps memory is allocated
    // dynamically
    time_map_ = new std::vector<warthog::mem::node_pool*>();
    for(uint32_t i = 0; i < 128; i++)
    {
        time_map_->push_back(new warthog::mem::node_pool(map_xy_sz_));
    }
}

warthog::cbs_ll_expansion_policy::~cbs_ll_expansion_policy()
{
    for(uint32_t i = 0; i < time_map_->size(); i++)
    {
        delete time_map_->at(i);
    }
    time_map_->clear();
    delete time_map_;
    delete cons_;
    delete neis_;
}


void 
warthog::cbs_ll_expansion_policy::expand(warthog::search_node* current,
		warthog::problem_instance* problem)
{
	reset();

    // get the xy id of the current node and extract current timestep
	uint32_t nodeid = (uint32_t)(current->get_id() & UINT32_MAX);
    uint32_t timestep = (uint32_t)(current->get_id() >> 32);

	// get adjacent grid tiles (bitpacked into one 32bit word)
	uint32_t tiles = 0;
	map_->get_neighbours(nodeid, (uint8_t*)&tiles);

    // neighbour ids are calculated using nodeid offsets
	uint32_t nid_m_w = nodeid - map_->width();
	uint32_t nid_p_w = nodeid + map_->width();

    // edge constraints for the current node
    cbs_constraint dummy;
    cbs_constraint* cur_cc = cons_->get_constraint(nodeid, timestep);

    // cardinal successors
    cbs_constraint* succ_cc = cons_->get_constraint(nid_m_w, timestep+1);
    if( ((tiles & 514) == 514) && // NORTH is not an obstacle
        (!cur_cc || !(cur_cc->e_ & warthog::grid::NORTH)) &&  // no edge constraint
        (!succ_cc || !succ_cc->v_) )  // no vertex constraint
	{  
		add_neighbour(__generate(nid_m_w, timestep+1), 1);
	} 

    succ_cc = cons_->get_constraint(nodeid + 1, timestep+1);
	if( ((tiles & 1536) == 1536) && // E
        (!cur_cc || !(cur_cc->e_ & warthog::grid::EAST)) &&
        (!succ_cc || !succ_cc->v_ ) )
	{
		add_neighbour(__generate(nodeid + 1, timestep+1), 1);
	}

    succ_cc = cons_->get_constraint(nid_p_w, timestep+1);
	if( ((tiles & 131584) == 131584) && // S
        (!cur_cc || !(cur_cc->e_ & warthog::grid::SOUTH)) && 
        (!succ_cc || !succ_cc->v_) )
	{ 

		add_neighbour(__generate(nid_p_w, timestep+1), 1);
	}

    succ_cc = cons_->get_constraint(nodeid - 1, timestep+1);
	if( ((tiles & 768) == 768) && // W
        (!cur_cc || !(cur_cc->e_ & warthog::grid::WEST)) && 
        (!succ_cc || !succ_cc->v_) )
	{ 
		add_neighbour(__generate(nodeid - 1, timestep+1), 1);
	}

    // wait successor
    succ_cc = cons_->get_constraint(nodeid, timestep+1);
    if( (!cur_cc || !(cur_cc->e_ & warthog::grid::WEST)) && 
        (!succ_cc || !succ_cc->v_) )
    {
        add_neighbour(__generate(nodeid, timestep+1), 1);
    }
}

void
warthog::cbs_ll_expansion_policy::get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy(nid & UINT32_MAX, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::cbs_ll_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    warthog::sn_id_t max_id = map_->header_width() * map_->header_height();
    if(pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return __generate(padded_id, 0);
}

warthog::search_node*
warthog::cbs_ll_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    warthog::sn_id_t max_id = map_->header_width() * map_->header_height();
    if(pi->target_id_ >= max_id) { return 0; }

    // precompute h-values
    h_->set_current_target(pi->target_id_);

    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    if(map_->get_label(padded_id) == 0) { return 0; }
    return __generate(padded_id, 0);
}

size_t
warthog::cbs_ll_expansion_policy::mem()
{
   size_t total = sizeof(*this) + map_->mem();
   size_t tm_sz = time_map_->size();
   for(uint32_t i = 0; i < tm_sz; i++)
   {
       total += time_map_->at(i)->mem();
   }
   total += sizeof(neighbour_record) * neis_->capacity();
   return total;
}
