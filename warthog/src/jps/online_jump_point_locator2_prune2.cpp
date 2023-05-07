#include "gridmap.h"
#include "jps.h"
#include "online_jump_point_locator2_prune2.h"
#include "global.h"

#include <cassert>
#include <climits>

typedef warthog::online_jump_point_locator2_prune2 jlp;
namespace G = global::query;
namespace S = global::statis;

jlp::online_jump_point_locator2_prune2(
gridmap* map,
online_jps_pruner2* pruner)
	: map_(map)//, jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap();
  jp = pruner;
	current_node_id_ = current_rnode_id_ = warthog::INF32;
	current_goal_id_ = current_rgoal_id_ = warthog::INF32;
}

jlp::~online_jump_point_locator2_prune2()
{
	delete rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// this version will be used when jumping North or South. 
warthog::gridmap*
jlp::create_rmap()
{
	uint32_t maph = map_->header_height();
	uint32_t mapw = map_->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::gridmap* rmap = new warthog::gridmap(rmaph, rmapw);

	for(uint32_t x = 0; x < mapw; x++) 
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			uint32_t label = map_->get_label(map_->to_padded_id(x, y));
			uint32_t rx = ((rmapw-1) - y);
			uint32_t ry = x;
			uint32_t rid = rmap->to_padded_id(rx, ry);
			rmap->set_label(rid, label);
		}
	}
	return rmap;
}


// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a 
// jump point successor.
//
// @return: the id of a jump point successor or warthog::INF if no jp exists.
void
jlp::jump(warthog::jps::direction d,
	   	uint32_t node_id, uint32_t goal_id, 
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	// cache node and goal ids so we don't need to convert all the time
	if(goal_id != current_goal_id_)
	{
		current_goal_id_ = goal_id;
		current_rgoal_id_ = map_id_to_rmap_id(goal_id);
	}

	if(node_id != current_node_id_)
	{
		current_node_id_ = node_id;
		current_rnode_id_ = map_id_to_rmap_id(node_id);
	}

	switch(d)
	{
		case warthog::jps::NORTH:
			jump_north(jpoints, costs);
			break;
		case warthog::jps::SOUTH:
			jump_south(jpoints, costs);
			break;
		case warthog::jps::EAST:
			jump_east(jpoints, costs);
			break;
		case warthog::jps::WEST:
			jump_west(jpoints, costs);
			break;
		case warthog::jps::NORTHEAST:
			jump_northeast(jpoints, costs);
			break;
		case warthog::jps::NORTHWEST:
			jump_northwest(jpoints, costs);
			break;
		case warthog::jps::SOUTHEAST:
			jump_southeast(jpoints, costs);
			break;
		case warthog::jps::SOUTHWEST:
			jump_southwest(jpoints, costs);
			break;
		default:
			break;
	}
}

void
jlp::jump_north(
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_north(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_);

	if(jumpnode_id != warthog::INF32)
	{
    uint32_t rjp_id = jumpnode_id;
		jumpnode_id = current_node_id_ - jp->jump_step * map_->width();
    // _backwards_gval_update(jumpnode_id, jumpcost, pa->get_g(), 1); // update south
    backwards_gval_update_NS(jumpnode_id, rjp_id, jumpcost, pa->get_g(), jps::SOUTH);
    jp->setup(jp->north, pa->get_g(), G::gval(jumpnode_id), jumpcost);

    if (pa->get_g() + jumpcost <= G::gval(jumpnode_id)) {
      // *(((uint8_t*)&jumpnode_id)+3) = warthog::jps::NORTH;
      jpoints.push_back(jumpnode_id);
      costs.push_back(jumpcost);
    }
	} else jp->north.deactivate();
}

void
jlp::__jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		warthog::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
jlp::jump_south(
		std::vector<uint32_t>& jpoints, 
		std::vector<warthog::cost_t>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_south(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_);

	if(jumpnode_id != warthog::INF32)
	{
    uint32_t rjp_id = jumpnode_id;
    jumpnode_id = current_node_id_ + jp->jump_step * map_->width();
    // _backwards_gval_update(jumpnode_id, jumpcost, pa->get_g(), 0); // update north
    backwards_gval_update_NS(jumpnode_id, rjp_id, jumpcost, pa->get_g(), jps::NORTH);
    jp->setup(jp->south, pa->get_g(), G::gval(jumpnode_id), jumpcost);

    if (pa->get_g() + jumpcost <= G::gval(jumpnode_id)) {
      // *(((uint8_t*)&jumpnode_id)+3) = warthog::jps::SOUTH;
      jpoints.push_back(jumpnode_id);
      costs.push_back(jumpcost);
    }
	} else jp->south.deactivate();
}

void
jlp::__jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		warthog::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
jlp::jump_east(
		std::vector<uint32_t>& jpoints, 
		std::vector<warthog::cost_t>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, map_);

	if(jumpnode_id != warthog::INF32)
	{
    // _backwards_gval_update(jumpnode_id, jumpcost, pa->get_g(), 3); // update west
    backwards_gval_update_EW(jumpnode_id, jumpcost, pa->get_g(), jps::WEST);
    jp->setup(jp->east, pa->get_g(), G::gval(jumpnode_id), jumpcost);
    if (pa->get_g() + jumpcost <= G::gval(jumpnode_id)) {
      // *(((uint8_t*)&jumpnode_id)+3) = warthog::jps::EAST;
      jpoints.push_back(jumpnode_id);
      costs.push_back(jumpcost);
    }
	} else jp->east.deactivate();
}


void
jlp::__jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
		warthog::gridmap* mymap)
{
	jumpnode_id = node_id;

	uint32_t neis[3] = {0, 0, 0};
	bool deadend = false;

	jumpnode_id = node_id;
	while(true)
	{
		// read in tiles from 3 adjacent rows. the curent node 
		// is in the low byte of the middle row
		mymap->get_neighbours_32bit(jumpnode_id, neis);

		// identity forced neighbours and deadend tiles. 
		// forced neighbours are found in the top or bottom row. they 
		// can be identified as a non-obstacle tile that follows
		// immediately  after an obstacle tile. A dead-end tile is
		// an obstacle found  on the middle row; 
		uint32_t 
		forced_bits = (~neis[0] << 1) & neis[0];
		forced_bits |= (~neis[2] << 1) & neis[2];
		uint32_t 
		deadend_bits = ~neis[1];

		// stop if we found any forced or dead-end tiles
		int stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint32_t stop_pos = __builtin_ffs(stop_bits)-1; // returns idx+1
			jumpnode_id += stop_pos; 
			deadend = deadend_bits & (1 << stop_pos);
			break;
		}

		// jump to the last position in the cache. we do not jump past the end
		// in case the last tile from the row above or below is an obstacle.
		// Such a tile, followed by a non-obstacle tile, would yield a forced 
		// neighbour that we don't want to miss.
		jumpnode_id += 31;
	}

  uint32_t& num_steps = jp->jump_step;
	num_steps = jumpnode_id - node_id;
#ifdef CNT
  S::scan_cnt += (num_steps >> 5) ;
#endif
	uint32_t goal_dist = goal_id - node_id;
	if(num_steps > goal_dist)
	{
    num_steps = goal_dist;
		jumpnode_id = goal_id;
		jumpcost = goal_dist;
		return;
	}

	if(deadend)
	{
		// number of steps to reach the deadend tile is not
		// correct here since we just inverted neis[1] and then
		// looked for the first set bit. need -1 to fix it.
		num_steps -= (1 && num_steps);
		jumpnode_id = warthog::INF32;
	}
	jumpcost = num_steps;
	
}

// analogous to ::jump_east 
void
jlp::jump_west(
		std::vector<uint32_t>& jpoints, 
		std::vector<warthog::cost_t>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, map_);

	if(jumpnode_id != warthog::INF32)
	{
    // _backwards_gval_update(jumpnode_id, jumpcost, pa->get_g(), 2); // update east
    backwards_gval_update_EW(jumpnode_id, jumpcost, pa->get_g(), jps::EAST);
    jp->setup(jp->west, pa->get_g(), G::gval(jumpnode_id), jumpcost);

    if (pa->get_g() + jumpcost <= G::gval(jumpnode_id)) {
      // *(((uint8_t*)&jumpnode_id)+3) = warthog::jps::WEST;
      jpoints.push_back(jumpnode_id);
      costs.push_back(jumpcost);
    }
	} else jp->west.deactivate();
}

void
jlp::__jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
		warthog::gridmap* mymap)
{
	bool deadend = false;
	uint32_t neis[3] = {0, 0, 0};

	jumpnode_id = node_id;
	while(true)
	{
		// cache 32 tiles from three adjacent rows.
		// current tile is in the high byte of the middle row
		mymap->get_neighbours_upper_32bit(jumpnode_id, neis);

		// identify forced and dead-end nodes
		uint32_t 
		forced_bits = (~neis[0] >> 1) & neis[0];
		forced_bits |= (~neis[2] >> 1) & neis[2];
		uint32_t 
		deadend_bits = ~neis[1];

		// stop if we encounter any forced or deadend nodes
		uint32_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint32_t stop_pos = __builtin_clz(stop_bits);
			jumpnode_id -= stop_pos;
			deadend = deadend_bits & (0x80000000 >> stop_pos);
			break;
		}
		// jump to the end of cache. jumping +32 involves checking
		// for forced neis between adjacent sets of contiguous tiles
		jumpnode_id -= 31;
	
	}

  uint32_t& num_steps = jp->jump_step;
	num_steps = node_id - jumpnode_id;
	uint32_t goal_dist = node_id - goal_id;
#ifdef CNT
  S::scan_cnt += (num_steps >> 5);
#endif
	if(num_steps > goal_dist)
	{
    num_steps = goal_dist;
		jumpnode_id = goal_id;
		jumpcost = goal_dist;
 		return;
	}

	if(deadend)
	{
		// number of steps to reach the deadend tile is not
		// correct here since we just inverted neis[1] and then
		// counted leading zeroes. need -1 to fix it.
		num_steps -= (1 && num_steps);
		jumpnode_id = warthog::INF32;
	}
	jumpcost = num_steps;
}

void
jlp::jump_northeast(
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 1542) != 1542) { return; }

  G::cur_diag_gval = pa->get_g();
  jp->v.init_before_diag(jps::NORTH, jps::EAST);
  jp->h.init_before_diag(jps::EAST, jps::NORTH);
  jp->setup(jp->v, jp->north.ga, jp->north.gb, jp->north.dC);
  jp->setup(jp->h, jp->east.ga, jp->east.gb, jp->east.dC);

	while(node_id != warthog::INF32)
	{
    jp1_id = jp2_id = warthog::INF32;
		__jump_northeast(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
      uint32_t rjp_id = jp1_id;
			jp1_id = node_id - (uint32_t)(jp1_cost) * map_->width();
      // update in south
      // _backwards_gval_update(jp1_id, jp1_cost, G::cur_diag_gval, 1);
      backwards_gval_update_NS(jp1_id, rjp_id, jp1_cost, G::cur_diag_gval, jps::SOUTH);
      if (G::cur_diag_gval+ jp1_cost <= G::gval(jp1_id)) {
				// *(((uint8_t*)&jp1_id)+3) = warthog::jps::NORTH;
        jpoints.push_back(jp1_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
      }
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
      cost_t gp = pa->get_g() + cost_to_nodeid + jumpcost;
      // update in west
      // _backwards_gval_update(jp2_id, jp2_cost, gp, 3);
      backwards_gval_update_EW(jp2_id, jp2_cost, gp, jps::WEST);
      if (gp + jp2_cost <= G::gval(jp2_id)) {
				// *(((uint8_t*)&jp2_id)+3) = warthog::jps::EAST;
        jpoints.push_back(jp2_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
      }
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
jlp::__jump_northeast(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
		uint32_t& jp_id1, warthog::cost_t& cost1,
		uint32_t& jp_id2, warthog::cost_t& cost2)
{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();

  if (jp->v.dominated() || jp->h.dominated()) {
    jumpnode_id = warthog::INF32; jumpcost = 0; return;
  }
	while(true)
	{
		num_steps++;
    G::cur_diag_gval += DBL_ROOT_TWO;
		node_id = node_id - mapw + 1;
		rnode_id = rnode_id + rmapw + 1;
    // if (iscorner[node_id])
    //   global::query::set_corner_gv(node_id, G::cur_diag_gval);

    if ((!jp->v.next()) || (!jp->h.next())) {
      jumpnode_id = warthog::INF32; jumpcost = 0; return;
    }
		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
    jp->before_scanv(rmap_, rnode_id, 1);
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
    jp->jumpcost = cost1;
    if (!jp->after_scanv(rmap_, node_id-jp->jump_step*mapw, jp_id1, cost1)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }


    jp->before_scanh(map_, node_id, 1);
		__jump_east(node_id, goal_id, jp_id2, cost2, map_);
    jp->jumpcost = cost2;
    if (!jp->after_scanh(map_, node_id+jp->jump_step, jp_id2, cost2)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }
		if((jp_id1 & jp_id2) != warthog::INF32) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) 
		{ 
			node_id = jp_id1 = jp_id2 = warthog::INF32; 
			break; 
		}

	}
#ifdef CNT
  S::scan_cnt += num_steps;
#endif
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
jlp::jump_northwest(
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 771) != 771) { return; }

  G::cur_diag_gval = pa->get_g();
  jp->v.init_before_diag(jps::NORTH, jps::WEST);
  jp->h.init_before_diag(jps::WEST, jps::NORTH);

  jp->setup(jp->v, jp->north.ga, jp->north.gb, jp->north.dC);
  jp->setup(jp->h, jp->west.ga, jp->west.gb, jp->west.dC);

	while(node_id != warthog::INF32)
	{
    jp1_id = jp2_id = warthog::INF32;
		__jump_northwest(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
      uint32_t rjp_id = jp1_id;
      jp1_id = node_id - (uint32_t)(jp1_cost) * map_->width();
      // update in south
      // _backwards_gval_update(jp1_id, jp1_cost, G::cur_diag_gval, 1);
      backwards_gval_update_NS(jp1_id, rjp_id, jp1_cost, G::cur_diag_gval, jps::SOUTH);
      if (G::cur_diag_gval + jp1_cost <= G::gval(jp1_id)) {
				// *(((uint8_t*)&jp1_id)+3) = warthog::jps::NORTH;
        jpoints.push_back(jp1_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
      }
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
      // update in east
      // _backwards_gval_update(jp2_id, jp2_cost, G::cur_diag_gval, 2);
      backwards_gval_update_EW(jp2_id, jp2_cost, G::cur_diag_gval, jps::EAST);
      if (G::cur_diag_gval + jp2_cost <= G::gval(jp2_id)) {
        // *(((uint8_t*)&jp2_id)+3) = warthog::jps::WEST;
        jpoints.push_back(jp2_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
      }
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
jlp::__jump_northwest(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		uint32_t& jp_id1, warthog::cost_t& cost1, 
		uint32_t& jp_id2, warthog::cost_t& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();

  if (jp->v.dominated() || jp->h.dominated()) {
    jumpnode_id = warthog::INF32; jumpcost = 0; return;
  }
	while(true)
	{
		num_steps++;
    G::cur_diag_gval += DBL_ROOT_TWO;
		node_id = node_id - mapw - 1;
		rnode_id = rnode_id - (rmapw - 1);
    // if (iscorner[node_id])
    //   global::query::set_corner_gv(node_id, G::cur_diag_gval);

    if ((!jp->v.next()) || (!jp->h.next())) {
      jumpnode_id = warthog::INF32; jumpcost = 0; return;
    }
		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
    jp->before_scanv(rmap_,  rnode_id, 1);
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
    jp->jumpcost = cost1;
    if (!jp->after_scanv(rmap_, node_id-jp->jump_step*mapw, jp_id1, cost1)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }

    jp->before_scanh(map_, node_id, -1);
		__jump_west(node_id, goal_id, jp_id2, cost2, map_);
    jp->jumpcost = cost2;
    if (!jp->after_scanh(map_, node_id-jp->jump_step, jp_id2, cost2)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }

		if((jp_id1 & jp_id2) != warthog::INF32) { break; }
		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { node_id = jp_id1 = jp_id2 = warthog::INF32; break; 
		}
	}
#ifdef CNT
  S::scan_cnt += num_steps;
#endif
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
jlp::jump_southeast(
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
  G::cur_diag_gval = pa->get_g();

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);
	
	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 394752) != 394752) { return; }

  G::cur_diag_gval = pa->get_g();
  jp->v.init_before_diag(jps::SOUTH, jps::EAST);
  jp->h.init_before_diag(jps::EAST, jps::SOUTH);
  jp->setup(jp->v, jp->south.ga, jp->south.gb, jp->south.dC);
  jp->setup(jp->h, jp->east.ga, jp->east.gb, jp->east.dC);

	while(node_id != warthog::INF32)
	{
    jp1_id = jp2_id = warthog::INF32;
		__jump_southeast(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
    {
      uint32_t rjp_id = jp1_id;
			jp1_id = node_id + (uint32_t)(jp1_cost) * map_->width();
      // update in north
      // _backwards_gval_update(jp1_id, jp1_cost, G::cur_diag_gval, 0);
      backwards_gval_update_NS(jp1_id, rjp_id, jp1_cost, G::cur_diag_gval, jps::NORTH);
      if (G::cur_diag_gval + jp1_cost <= G::gval(jp1_id)) {
        // *(((uint8_t*)&jp1_id)+3) = warthog::jps::SOUTH;
        jpoints.push_back(jp1_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
      }
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
      // update in west
      // _backwards_gval_update(jp2_id, jp2_cost, G::cur_diag_gval, 3);
      backwards_gval_update_EW(jp2_id, jp2_cost, G::cur_diag_gval, jps::WEST);
      if (G::cur_diag_gval + jp2_cost <= G::gval(jp2_id)) {
        // *(((uint8_t*)&jp2_id)+3) = warthog::jps::EAST;
        jpoints.push_back(jp2_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
      }
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
jlp::__jump_southeast(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		uint32_t& jp_id1, warthog::cost_t& cost1, 
		uint32_t& jp_id2, warthog::cost_t& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();

  if (jp->v.dominated() || jp->h.dominated()) {
    jumpnode_id = warthog::INF32; jumpcost = 0; return;
  }
	while(true)
	{
		num_steps++;
    G::cur_diag_gval += DBL_ROOT_TWO;
		node_id = node_id + mapw + 1;
		rnode_id = rnode_id + rmapw - 1;
    // if (iscorner[node_id])
    //   global::query::set_corner_gv(node_id, G::cur_diag_gval);

    if ((!jp->v.next()) || (!jp->h.next())) {
      jumpnode_id = warthog::INF32; jumpcost = 0; return;
    }
		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
    jp->before_scanv(rmap_, rnode_id, -1);
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
    jp->jumpcost = cost1;
    if (!jp->after_scanv(rmap_, node_id+jp->jump_step*mapw, jp_id1, cost1)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }

    jp->before_scanh(map_, node_id, 1);
		__jump_east(node_id, goal_id, jp_id2, cost2, map_);
    jp->jumpcost = cost2;
    if (!jp->after_scanh(map_, node_id+jp->jump_step, jp_id2, cost2)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }
    if ((jp_id1 & jp_id2) != warthog::INF32) break;
		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { node_id = jp_id1 = jp_id2 = warthog::INF32; break; 
		}
	}
#ifdef CNT
  S::scan_cnt += num_steps;
#endif
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
jlp::jump_southwest(
		std::vector<uint32_t>& jpoints,
		std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
  G::cur_diag_gval = pa->get_g();
	
	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early termination (first step is invalid)
	if((neis & 197376) != 197376) { return; }

  G::cur_diag_gval = pa->get_g();
  jp->v.init_before_diag(jps::SOUTH, jps::WEST);
  jp->h.init_before_diag(jps::WEST, jps::SOUTH);

  jp->setup(jp->v, jp->south.ga, jp->south.gb, jp->south.dC);
  jp->setup(jp->h, jp->west.ga, jp->west.gb, jp->west.dC);

	while(node_id != warthog::INF32)
	{

    jp1_id = jp2_id = warthog::INF32;
		__jump_southwest(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, 
				jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
      uint32_t rjp_id = jp1_id;
      jp1_id = node_id + (uint32_t)(jp1_cost) * map_->width();
      // update in north
      // _backwards_gval_update(jp1_id, jp1_cost, G::cur_diag_gval, 0);
      backwards_gval_update_NS(jp1_id, rjp_id, jp1_cost, G::cur_diag_gval, jps::NORTH);
      if (G::cur_diag_gval + jp1_cost <= G::gval(jp1_id)) {
        // *(((uint8_t*)&jp1_id)+3) = warthog::jps::SOUTH;
        jpoints.push_back(jp1_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
      }
			if(jp2_cost == 0) { break; }
		}

		if(jp2_id != warthog::INF32)
		{
      // update in east
      // _backwards_gval_update(jp2_id, jp2_cost, G::cur_diag_gval, 2);
      backwards_gval_update_EW(jp2_id, jp2_cost, G::cur_diag_gval, jps::EAST);
      if (G::cur_diag_gval + jp2_cost <= G::gval(jp2_id)) {
        // *(((uint8_t*)&jp2_id)+3) = warthog::jps::WEST;
        jpoints.push_back(jp2_id);
        costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
      }
			if(jp1_cost == 0) { break; }
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
jlp::__jump_southwest(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		uint32_t& jp_id1, warthog::cost_t& cost1, 
		uint32_t& jp_id2, warthog::cost_t& cost2)
{
	// jump a single step (no corner cutting)
	uint32_t num_steps = 0;
	uint32_t mapw = map_->width();
	uint32_t rmapw = rmap_->width();

  if (jp->v.dominated() || jp->h.dominated()) {
    jumpnode_id = warthog::INF32; jumpcost = 0; return;
  }
	while(true)
	{
		num_steps++;
    G::cur_diag_gval += DBL_ROOT_TWO;
		node_id = node_id + mapw - 1;
		rnode_id = rnode_id - (rmapw + 1);
    // if (iscorner[node_id])
    //   global::query::set_corner_gv(node_id, G::cur_diag_gval);

    if ((!jp->v.next()) || (!jp->h.next())) {
      jumpnode_id = warthog::INF32; jumpcost = 0; return;
    }
		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
    jp->before_scanv(rmap_, rnode_id, -1);
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
    jp->jumpcost = cost1;
    if (!jp->after_scanv(rmap_, node_id+jp->jump_step*mapw, jp_id1, cost1)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }

    jp->before_scanh(map_, node_id, -1);
		__jump_west(node_id, goal_id, jp_id2, cost2, map_);
    jp->jumpcost = cost2;
    if (!jp->after_scanh(map_, node_id-jp->jump_step, jp_id2, cost2)) {
      jp_id1 = jp_id2 = jumpnode_id = INF32;
      jumpcost = 0; return;
    }

		if((jp_id1 & jp_id2) != warthog::INF32) { break; }
		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { node_id = jp_id1 = jp_id2 = warthog::INF32; break; 
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}
