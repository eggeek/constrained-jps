#ifndef WARTHOG_FORWARD_H
#define WARTHOG_FORWARD_H

// ./memory/forward.h
//
// Forward declarations
//
// @author: dharabor
// @created: 2018-11-07
//

#include <cstdint>

namespace warthog
{

class apriori_filter;
class apex_filter;
class bb_filter;
class cbs_ll_heuristic;
class dummy_filter;
class dummy_listener;
class expansion_policy;
class euclidean_heuristic;
class gridmap;
class gridmap_expansion_policy;
class problem_instance;
class search_node;
class solution;
class zero_heuristic;

template<typename H, typename E, typename Q, typename L>
class flexible_astar;

template<typename FILTER>
class graph_expansion_policy;

namespace graph
{

class node;

template<typename T_LABEL>
class edge_base;
typedef edge_base<uintptr_t> edge;

template<class T_NODE, class T_EDGE>
class xy_graph_base;
typedef xy_graph_base<node, edge> xy_graph;

}

namespace label
{

class af_labelling;
class bb_labelling;
class bbaf_labelling;
class dfs_labelling;
class firstmove_labelling;

}

namespace jps
{

}

namespace mem
{
}

namespace cbs
{
}

namespace util
{

class workload_manager;

}

}

#endif
