#include "gridmap.h"
#include "jps.h"
#include "sipp/jpst_locator.h"

#include <cassert>
#include <climits>

warthog::jpst_locator::jpst_locator(warthog::jpst_gridmap* jpst_map)
	: jpst_gm_(jpst_map)
{
    fc_jpl_ = new warthog::four_connected_jps_locator(jpst_gm_->gm_);
}

warthog::jpst_locator::~jpst_locator()
{ }


// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a 
// jump point successor.
//
// @return: the id of a jump point successor or warthog::INF32 if no jp exists.
void
warthog::jpst_locator::jump(warthog::jps::direction d,
	   	uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id, 
		double& jumpcost)
{
	switch(d)
	{
		case warthog::jps::NORTH:
			jump_north(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::SOUTH:
			jump_south(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::EAST:
			jump_east(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::WEST:
			jump_west(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		default:
			break;
	}
}

void
warthog::jpst_locator::jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
    uint32_t mapw = jpst_gm_->gm_->width();

    uint32_t jp_w_id;
    uint32_t jp_e_id;
    double jp_w_cost;
    double jp_e_cost;

    uint32_t next_id = node_id += mapw;
    uint32_t num_steps = 1;
    while(true)
    {
        // verify the next location is traversable
		if(!jpst_gm_->gm_->get_label(next_id))
        {
            next_id = warthog::INF32; 
            break; 
        }

        // stop if the next location has temporal obstacles;
        // take one step back and generate a jump point
		if(jpst_gm_->t_gm_->get_label(next_id))
        { 
        //    next_id -= mapw; 
        //    num_steps--;
        //    assert(num_steps > 0);
            break;
        }

        jump_east(next_id, goal_id, jp_e_id, jp_e_cost);
		if(jp_e_id != warthog::INF32) { break; }
        jump_west(next_id, goal_id, jp_w_id, jp_w_cost);
		if(jp_w_id != warthog::INF32) { break; }

        next_id += mapw;
        num_steps++;

    }
    jumpnode_id = next_id;
    jumpcost = num_steps;
    
    // adjust num_steps if we stopped due to a deadend 
    // (we return the distance to the last traversable tile)
    num_steps -= (1 * (next_id == warthog::INF32));
}


void
warthog::jpst_locator::jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
    uint32_t mapw = jpst_gm_->gm_->width();

    uint32_t jp_w_id;
    uint32_t jp_e_id;
    double jp_w_cost;
    double jp_e_cost;

    uint32_t next_id = node_id -= mapw;
    uint32_t num_steps = 1;
    while(true)
    {
        // stop if the current location is a deadend
		if(!jpst_gm_->gm_->get_label(next_id))
        { 
            next_id = warthog::INF32; break; 
        }

        // stop if the current location has temporal obstacles;
        // take one step back and generate a jump point
		if(jpst_gm_->t_gm_->get_label(next_id))
        { 
        //    num_steps--; 
        //    next_id += mapw; 
        //    assert(num_steps > 0);
            break; 
        }

        jump_east(next_id, goal_id, jp_e_id, jp_e_cost);
		if(jp_e_id != warthog::INF32) { break; }
        jump_west(next_id, goal_id, jp_w_id, jp_w_cost);
		if(jp_w_id != warthog::INF32) { break; }

        next_id -= mapw;
        num_steps++;

    }

    jumpnode_id = next_id;
    jumpcost = num_steps;

    // adjust num_steps if we stopped due to a deadend 
    // (we return the distance to the last traversable tile)
    num_steps -= (1 * (next_id == warthog::INF32));
}

void
warthog::jpst_locator::jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
   
   uint32_t fc_jp_id;
   double fc_jp_cost;

   // jump; look for spatial jump points
   fc_jpl_->jump_east(node_id, goal_id, fc_jp_id, fc_jp_cost);

    // jump again; this time look for temporal jump points
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, 
                (uint32_t)fc_jp_cost);

    // if no temporal jump point exists, return the 
    // result of the spatial jump
    if(jumpnode_id == warthog::INF32)
    {
        jumpnode_id = fc_jp_id;
        jumpcost = fc_jp_cost;
    }
}

// analogous to ::jump_east 
void
warthog::jpst_locator::jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
   uint32_t fc_jp_id;
   double fc_jp_cost;

   // jump; look for spatial jump points
   fc_jpl_->jump_west(node_id, goal_id, fc_jp_id, fc_jp_cost);

    // jump again; this time look for temporal jump points
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, 
                (uint32_t)fc_jp_cost);

    // if no temporal jump point exists, return the 
    // result of the spatial jump
    if(jumpnode_id == warthog::INF32)
    {
        jumpnode_id = fc_jp_id;
        jumpcost = fc_jp_cost;
    }
}

void
warthog::jpst_locator::__jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
		uint32_t jumplimit)
{
    uint32_t max_id = node_id + jumplimit;
	uint32_t neis[3] = {0, 0, 0};
	jumpnode_id = node_id + 1;

    uint8_t tmp[3];
    jpst_gm_->t_gm_->get_neighbours(node_id, tmp);
    if((tmp[0] | tmp[2]) & 1) 
    { 
        jumpcost = jumpnode_id - node_id;
        if(jumpnode_id > max_id)
        {
            jumpnode_id = warthog::INF32;
        }
        return;
    }

	while(jumpnode_id <= max_id)
	{
		// read in tiles from 3 adjacent rows. the curent node 
		// is in the low byte of the middle row
		jpst_gm_->t_gm_->get_neighbours_32bit(jumpnode_id, neis);

		// stop if we try to jump over nodes with temporal events
        // or which have neighbours with temporal events.
        // we treat such nodes as jump points
		uint32_t stop_bits = neis[0] | neis[1] | neis[2];
		if(stop_bits)
		{
			uint32_t stop_pos = (uint32_t)__builtin_ffs((int)stop_bits)-1; // returns idx+1
			jumpnode_id += (uint32_t)stop_pos; 
			break;
		}
		jumpnode_id += 32;
	}

    // return the jump point
    jumpcost = jumpnode_id - node_id;
    if(jumpnode_id > max_id)
    {
        jumpnode_id = warthog::INF32;
    }
}

void
warthog::jpst_locator::__jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
        uint32_t jumplimit)
{
    uint32_t min_id = node_id - jumplimit;
	uint32_t neis[3] = {0, 0, 0};
	jumpnode_id = node_id - 1;

    uint8_t tmp[3];
    jpst_gm_->t_gm_->get_neighbours(node_id, tmp);
    if((tmp[0] | tmp[2]) & 4) 
    { 
        // return the jump point
        jumpcost = node_id - jumpnode_id;
        if(jumpnode_id < min_id)
        {
            jumpnode_id = warthog::INF32;
        }
        return;
    }

	while(jumpnode_id >= min_id)
	{
		// cache 32 tiles from three adjacent rows.
		// current tile is in the high byte of the middle row
		jpst_gm_->t_gm_->get_neighbours_upper_32bit(jumpnode_id, neis);

		// stop if we try to jump over nodes with temporal events
        // or which have neighbours with temporal events.
        // we treat such nodes as jump points
		uint32_t stop_bits = neis[0] | neis[1] | neis[2];
		if(stop_bits)
		{
			uint32_t stop_pos = (uint32_t)__builtin_clz(stop_bits);
			jumpnode_id -= stop_pos;
			break;
		}
		jumpnode_id -= 32;
	}

    // return the jump point
    jumpcost = node_id - jumpnode_id;
    if(jumpnode_id < min_id)
    {
        jumpnode_id = warthog::INF32;
    }
}

