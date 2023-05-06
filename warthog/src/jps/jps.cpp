#include "constants.h"
#include "gridmap.h"
#include "jps.h"
#include "online_jump_point_locator2.h"
#include "xy_graph.h"

// computes the forced neighbours of a node.
// for a neighbour to be forced we must check that 
// (a) the alt path from the parent is blocked and
// (b) the neighbour is not an obstacle.
// if the test succeeds, we set a bit to indicate 
// the direction from the current node (tiles[4])
// to the forced neighbour.
//
// @return an integer value whose lower 8 bits indicate
// the directions of forced neighbours
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18 
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation. 
// We use the revised forced neighbour rules described in
// [Harabor and Grastien, The JPS Pathfinding System, SoCS, 2012]
// These rules do not allow diagonal transitions that cut corners.
uint32_t
warthog::jps::compute_forced(warthog::jps::direction d, uint32_t tiles)
{
	// NB: to avoid branching statements, bit operations are
	// used below to determine which neighbours are traversable
    // and which are obstacles
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
        {
            uint32_t branch_nw = ((tiles & 65792) == 256);
            ret |= (branch_nw << 3); // force west
            ret |= (branch_nw << 5); // force northwest

            uint32_t branch_ne = ((tiles & 263168) == 1024);
            ret |= (branch_ne << 2); // force east
            ret |= (branch_ne << 4); // force northeast
			break;
        }
		case warthog::jps::SOUTH:
        {
            uint32_t branch_sw = ((tiles & 257) == 256);
            ret |= (branch_sw << 3); // force west
            ret |= (branch_sw << 7); // force southwest

            uint32_t branch_se = ((tiles & 1028) == 1024);
            ret |= (branch_se << 2); // force east
            ret |= (branch_se << 6); // force southeast
            break;
        }
		case warthog::jps::EAST:
        {
			uint32_t branch_ne = ((tiles & 3) == 2);
            ret |= branch_ne;        // force north
            ret |= (branch_ne << 4); // force northeast

            uint32_t branch_se= ((tiles & 196608) == 131072);
            ret |= (branch_se << 1); // force south
            ret |= (branch_se << 6); // force southeast
			break;
        }
		case warthog::jps::WEST:
        {
            uint32_t force_nw = ((tiles & 6) == 2);
            ret |= force_nw;        // force north
            ret |= (force_nw << 5); // force northwest

			uint32_t force_sw = ((tiles & 393216) == 131072);
            ret |= (force_sw << 1); // force south
            ret |= (force_sw << 7); // force southwest
			break;
        }
		default:
			break;
	}
	return ret;
}

// Computes the natural neighbours of a node. 
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18 
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation. 
uint32_t 
warthog::jps::compute_natural(warthog::jps::direction d, uint32_t tiles)
{
	// In the shift operations below the constant values
	// correspond to bit offsets for warthog::jps::direction
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
			ret |= (uint32_t)((tiles & 2) == 2) << 0;
			break;
		case warthog::jps::SOUTH:
			ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
			break;
		case warthog::jps::EAST: 
			ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
			break;
		case warthog::jps::WEST:
			ret |= (uint32_t)((tiles & 256) == 256) << 3;
			break;
		case warthog::jps::NORTHWEST:
			ret |= (uint32_t)((tiles & 2) == 2) << 0;
			ret |= (uint32_t)((tiles & 256) == 256) << 3;
			ret |= (uint32_t)((tiles & 259) == 259) << 5;
			break;
		case warthog::jps::NORTHEAST:
			ret |= (uint32_t)((tiles & 2) == 2) << 0;
			ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
			ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
			break;
		case warthog::jps::SOUTHWEST:
			ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
			ret |= (uint32_t)((tiles & 256) == 256) << 3;
			ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
			break;
		case warthog::jps::SOUTHEAST:
			ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
			ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
			ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
			break;
		default:
			ret |= (uint32_t)((tiles & 2) == 2) << 0;
			ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
			ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
			ret |= (uint32_t)((tiles & 256) == 256) << 3;
			ret |= (uint32_t)((tiles & 259) == 259) << 5;
			ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
			ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
			ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
			break;
	}
	return ret;
}

warthog::graph::xy_graph*
warthog::jps::create_jump_point_graph(warthog::gridmap* gm) 
{
    warthog::graph::xy_graph* graph = new warthog::graph::xy_graph();
    warthog::jps::online_jump_point_locator2 jpl(gm);
    uint32_t mapwidth = gm->header_width();
    uint32_t mapheight = gm->header_height();
    std::unordered_map<uint32_t, uint32_t> id_map;

    // add nodes to graph
    for(uint32_t y = 0; y < mapheight; y++)
    {
        for(uint32_t x = 0; x < mapwidth; x++)
        {
            uint32_t from_id = gm->to_padded_id(y*mapwidth+x);
            if(!gm->get_label(gm->to_padded_id(x, y))) { continue; } 

            uint32_t w_id = from_id - 1;
            uint32_t e_id = from_id + 1;
            uint32_t s_id = from_id + gm->width();
            uint32_t n_id = from_id - gm->width();
            uint32_t nw_id = (from_id - gm->width()) - 1;
            uint32_t ne_id = (from_id - gm->width()) + 1;
            uint32_t sw_id = (from_id + gm->width()) - 1;
            uint32_t se_id = (from_id + gm->width()) + 1;
            
            // detect all corner turning points (== jump points) 
            // and add them to the jump point graph
            uint32_t tiles;
            gm->get_neighbours(from_id, (uint8_t*)&tiles);
            if( (!gm->get_label(nw_id) && 
                    gm->get_label(w_id) && gm->get_label(n_id)) ||
                (!gm->get_label(ne_id) && 
                    gm->get_label(e_id) && gm->get_label(n_id)) ||
                (!gm->get_label(se_id) && 
                    gm->get_label(e_id) && gm->get_label(s_id)) ||
                (!gm->get_label(sw_id) && 
                    gm->get_label(w_id) && gm->get_label(s_id)) )
            {
                uint32_t graph_id = graph->add_node((int32_t)x, (int32_t)y);
                id_map.insert(
                        std::pair<uint32_t, uint32_t>(from_id, graph_id));
            }
        }
    }

    // add edges to graph
    for(uint32_t from_id = 0; from_id < graph->get_num_nodes(); from_id++)
    {
        int32_t x, y;
        graph->get_xy(from_id, x, y);
        uint32_t gm_id = gm->to_padded_id((uint32_t)y*mapwidth+(uint32_t)x);
        warthog::graph::node* from = graph->get_node(from_id);

        for(uint32_t i = 0; i < 8; i++)
        {
            warthog::jps::direction d = (warthog::jps::direction)(1 << i);
            std::vector<uint32_t> jpoints;
            std::vector<double> jcosts;
            jpl.jump(d, gm_id, warthog::INF32, jpoints, jcosts);
            for(uint32_t idx = 0; idx < jpoints.size(); idx++)
            {
                uint32_t jp_id = jpoints[idx] & ((1 << 24) - 1);
                //warthog::jps::direction d = (warthog::jps::direction)(jpoints[idx] >> 24);
                std::unordered_map<uint32_t, uint32_t>::iterator it_to_id; 
                it_to_id = id_map.find(jp_id);
                assert(it_to_id != id_map.end());
                uint32_t to_id = it_to_id->second;
                warthog::graph::node* to = graph->get_node(to_id);
                from->add_outgoing(warthog::graph::edge(to_id, (uint32_t)jcosts[idx]));
                to->add_outgoing(warthog::graph::edge(from_id, (uint32_t)jcosts[idx]));
            }
        }
    }
    return graph;
}

warthog::gridmap*
warthog::jps::create_corner_map(warthog::gridmap* gm)
{
    uint32_t mapwidth = gm->header_width();
    uint32_t mapheight = gm->header_height();
    warthog::gridmap* corner_map = new warthog::gridmap(mapheight, mapwidth);

    // add nodes to graph
    for(uint32_t y = 0; y < mapheight; y++)
    {
        for(uint32_t x = 0; x < mapwidth; x++)
        {
            uint32_t from_id = gm->to_padded_id(y*mapwidth+x);
            if(!gm->get_label(from_id)) { continue; } 

            uint32_t w_id = from_id - 1;
            uint32_t e_id = from_id + 1;
            uint32_t s_id = from_id + gm->width();
            uint32_t n_id = from_id - gm->width();
            uint32_t nw_id = (from_id - gm->width()) - 1;
            uint32_t ne_id = (from_id - gm->width()) + 1;
            uint32_t sw_id = (from_id + gm->width()) - 1;
            uint32_t se_id = (from_id + gm->width()) + 1;
            
            // detect all corner turning points (== jump points) 
            // and add them to the jump point graph
            uint32_t tiles;
            gm->get_neighbours(from_id, (uint8_t*)&tiles);
            if( (!gm->get_label(nw_id) && 
                    gm->get_label(w_id) && gm->get_label(n_id)) ||
                (!gm->get_label(ne_id) && 
                    gm->get_label(e_id) && gm->get_label(n_id)) ||
                (!gm->get_label(se_id) && 
                    gm->get_label(e_id) && gm->get_label(s_id)) ||
                (!gm->get_label(sw_id) && 
                    gm->get_label(w_id) && gm->get_label(s_id)) )
            {
                corner_map->set_label(from_id, true);
            }
        }
    }
    return corner_map;
}

//warthog::jps::direction
//warthog::jps::compute_direction(
//        uint32_t px, uint32_t py, uint32_t x, uint32_t y)
//{
//    warthog::jps::direction dir = warthog::jps::NONE;
//    if(y == py)
//    {
//        if(x > px)
//            dir = warthog::jps::EAST;
//        else
//            dir = warthog::jps::WEST;
//    }
//    else if(y < py)
//    {
//        if(x == px)
//            dir = warthog::jps::NORTH;
//        else if(x < px)
//        {
//            dir = warthog::jps::NORTHWEST;
//            //dir = warthog::jps::WEST;
//        }
//        else 
//            dir = warthog::jps::NORTHEAST;
//            //dir = warthog::jps::EAST;
//    }
//    else // y > py
//    {
//        if(x == px)
//            dir = warthog::jps::SOUTH;
//        else if(x < px)
//        {
//            dir = warthog::jps::SOUTHWEST;
//           // dir = warthog::jps::WEST;
//        }
//        else // x > px
//            dir = warthog::jps::SOUTHEAST;
//            //dir = warthog::jps::EAST;
//    }
//    assert(dir != warthog::jps::NONE);
//    return dir;
//}
