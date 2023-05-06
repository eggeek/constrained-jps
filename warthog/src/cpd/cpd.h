// cpd/cpd.h
//
// Various defintions / data-structures for CPDs
//
// @author: dharabor
// @created: 2020-02-25
//

#ifndef WARTHOG_CPD_CPD_H
#define WARTHOG_CPD_CPD_H

#include <cassert>
#include <functional>
#include <iostream>
#include <vector>

#include "flexible_astar.h"
#include "helpers.h"
#include "search_node.h"
#include "timer.h"
#include "zero_heuristic.h"

namespace warthog
{
namespace cpd
{

enum clock_direction {CW, CCW};

// defines a 32bit run in a run-length encoding
// the first 4 bits tell the symbol.
// the next 28 bits tell the index where the run begins
struct rle_run32
{
    uint8_t 
    get_move() { return data_ & 0xF; } 

    uint32_t 
    get_index() { return data_ >> 4; } 


    void
    print(std::ostream& out)
    {
        out << " [" << get_index() << ", " << get_move() << "]";
    }

    uint32_t data_;
};

std::istream&
operator>>(std::istream& in, warthog::cpd::rle_run32& the_run);

std::ostream&
operator<<(std::ostream& out, warthog::cpd::rle_run32& the_run);

//  limits on the number of nodes in a graph 
//  for which we compute a CPD
static const uint32_t RLE_RUN32_MAX_INDEX = (UINT32_MAX >> 4);

// limits on the maximum number of first-move labels that need to be stored.
// this value should be greater than the maximum degree of any node plus one
// extra value for the case where a node is unreachable
static const uint32_t CPD_FM_MAX = 16;

// special value to denote that no first move exists.
static const uint32_t CPD_FM_NONE=0xF;

// a collection of optimal first moves. 
// we keep one bit for each optimal move. the maximum
// degree of any node is determined by CPD_FM_MAX
typedef uint16_t fm_coll;

// a DFS pre-order traversal of the input graph is known to produce an 
// effective node order for the purposes of compression
// @param g: the input graph
// @param column_order: a list of node ids as visited by DFS from a random
// start node
void
compute_dfs_preorder(
        warthog::graph::xy_graph* g,
        std::vector<uint32_t>* column_order,
        uint32_t seed=0);

}

}

#endif

