#ifndef WARTHOG_CPD_GRAPH_EXPANSION_POLICY_H
#define WARTHOG_CPD_GRAPH_EXPANSION_POLICY_H

// cpd/cpd_graph_expansion_policy.h
//
// CPD-based pathfinding. The only successors are those which a CPD oracle 
// tells are on the optimal path, from the currently expanded node to the 
// target.
//
// @author: dharabor
// @created: 2020-03-02
//

#include "expansion_policy.h"
#include "graph_oracle.h"
#include "search_node.h"

namespace warthog
{

class cpd_graph_expansion_policy  : public expansion_policy
{
    public:

        cpd_graph_expansion_policy(warthog::cpd::graph_oracle* oracle) 
           : expansion_policy(oracle->get_graph()->get_num_nodes()), 
             oracle_(oracle) 
        { }

        virtual ~cpd_graph_expansion_policy()
        { }

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);
      
        virtual void
        get_xy(sn_id_t node_id, int32_t& x, int32_t& y);

    private:
        warthog::cpd::graph_oracle* oracle_;

};

}

#endif

