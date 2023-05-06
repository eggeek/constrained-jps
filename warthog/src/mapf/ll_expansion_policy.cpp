#include "cbs.h"
#include "grid.h"
#include "helpers.h"
#include "ll_expansion_policy.h"
#include "problem_instance.h"
#include "time_constraints.h"

#include <algorithm>

using namespace warthog::cbs;

warthog::ll_expansion_policy::ll_expansion_policy(
		warthog::gridmap* map, warthog::cbs_ll_heuristic* h) : map_(map), h_(h)
{
    neis_ = new warthog::arraylist<neighbour_record>(32);

    map_xy_sz_ = map->height() * map->width();
    assert(map_xy_sz_ > 0);

    cons_ = new warthog::mapf::time_constraints<warthog::mapf::cell_constraint>
             (map_xy_sz_);

    // preallocate memory for search nodes up to some number of timesteps 
    // in advance. for subsequent timesteps memory is allocated
    // dynamically
    search_node_pool_ = new std::vector<warthog::mem::node_pool*>();
    for(uint32_t i = 0; i < 128; i++)
    {
        search_node_pool_->push_back(new warthog::mem::node_pool(map_xy_sz_));
    }
}

warthog::ll_expansion_policy::~ll_expansion_policy()
{
    for(uint32_t i = 0; i < search_node_pool_->size(); i++)
    {
        delete search_node_pool_->at(i);
    }
    search_node_pool_->clear();
    delete search_node_pool_;
    delete cons_;
    delete neis_;
}


void 
warthog::ll_expansion_policy::expand(warthog::search_node* current,
		warthog::problem_instance* problem)
{
	reset();

    // get the xy id of the current node and extract current timestep
	uint32_t xy_id = (uint32_t)current->get_id();
    uint32_t timestep = (uint32_t)(current->get_id() >> 32);

    // neighbour ids are calculated using xy_id offsets
	uint32_t nid_m_w = xy_id - map_->width();
	uint32_t nid_p_w = xy_id + map_->width();

    // edge constraints for the current node 
    warthog::mapf::cell_constraint* cur_cc = 
        cons_->get_constraint(xy_id, timestep);

    // move NORTH
    double move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::NORTH] : 1;
    if( map_->get_label(nid_m_w) && move_cost != warthog::INF32 )
    {
        add_neighbour( __generate(nid_m_w, timestep+1), move_cost );
            
	} 

    // move EAST
    move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::EAST] : 1;
    if( map_->get_label(xy_id + 1) && move_cost != warthog::INF32)
    {
        add_neighbour( __generate(xy_id+1, timestep+1), move_cost );
	} 

    // move SOUTH
    move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::SOUTH] : 1;
    if( map_->get_label(nid_p_w) && move_cost != warthog::INF32 )
    {
        add_neighbour( __generate(nid_p_w, timestep+1), move_cost );
	} 

    // move WEST
    move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::WEST] : 1;
    if( map_->get_label(xy_id - 1) && move_cost != warthog::INF32 )
    {
        add_neighbour( __generate(xy_id-1, timestep+1), move_cost );
            
	} 

    // move WAIT
    move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::WAIT] : 1;
    if( move_cost != warthog::INF32 )
    {
        add_neighbour( __generate(xy_id, timestep+1), move_cost );
	} 
}

void
warthog::ll_expansion_policy::get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)nid, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::ll_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    warthog::sn_id_t max_id = map_->header_width() * map_->header_height();
    if(pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    return __generate(padded_id, 0);
}

warthog::search_node*
warthog::ll_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    warthog::sn_id_t max_id = map_->header_width() * map_->header_height();
    if(pi->target_id_ >= max_id) { return 0; }

    h_->set_current_target(pi->target_id_);
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    return __generate(padded_id, 0);
}

size_t
warthog::ll_expansion_policy::mem()
{
   size_t total = sizeof(*this) + map_->mem();
   size_t tm_sz = search_node_pool_->size();
   for(uint32_t i = 0; i < tm_sz; i++)
   {
       total += search_node_pool_->at(i)->mem();
   }
   total += sizeof(neighbour_record) * neis_->capacity();
   return total;
}
