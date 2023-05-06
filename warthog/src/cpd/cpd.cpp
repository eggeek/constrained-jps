#include "cpd.h"
#include "xy_graph.h"


void
warthog::cpd::compute_dfs_preorder(
        warthog::graph::xy_graph* g,
        std::vector<uint32_t>* column_order,
        uint32_t dfs_seed)
{
    assert(g->get_num_nodes() < UINT32_MAX);

    // label each vertex with an number to tell when it was processed by DFS
    std::vector<uint32_t> dfs_labeling(g->get_num_nodes(), UINT32_MAX);
    typedef std::pair<uint32_t, uint32_t> dfs_pair;
    std::vector<dfs_pair> dfs_stack;

    uint32_t preorder_id = 0;
    dfs_stack.push_back(dfs_pair(dfs_seed, 0));
    dfs_stack.reserve(g->get_num_nodes());
    while(dfs_stack.size())
    {
        // current node in the dfs tree 
        dfs_pair& dfs_node = dfs_stack.back();

        warthog::graph::node* n = g->get_node(dfs_node.first);
        if(dfs_labeling.at(dfs_node.first) == UINT32_MAX)
        { 
            // first time we reach this node. assign it an id
            dfs_labeling.at(dfs_node.first) = preorder_id++;
        }

        while(true)
        {
            warthog::graph::edge_iter begin = n->outgoing_begin() + dfs_node.second++;
            if(begin >= n->outgoing_end())
            {
                // current node has no more unreached successors. backtrack.
                dfs_stack.pop_back();
                break;
            }

            assert(begin->node_id_ < g->get_num_nodes());
            if(dfs_labeling.at(begin->node_id_) == UINT32_MAX)
            {
                // otherwise, traverse down the next branch
                dfs_stack.push_back(dfs_pair(begin->node_id_, 0));
                break;
            }
        }
    }

    // unreachable nodes are added to the end of the 
    // column ordering
    uint32_t unreachable = 0;
    for(uint32_t i = 0; i < dfs_labeling.size(); i++)
    {
        if(dfs_labeling.at(i) == UINT32_MAX)
        {
            dfs_labeling.at(i) = preorder_id++;
            unreachable++;
        }
    }
    assert(preorder_id == g->get_num_nodes());
    std::cerr << "graph has " << unreachable << " unreachable nodes\n";

    // sort the columns by their dfs labels (smallest to largest)
    column_order->resize(g->get_num_nodes());
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        column_order->at(dfs_labeling.at(i)) = i;
    }
}

std::istream&
warthog::cpd::operator>>(std::istream& in, warthog::cpd::rle_run32& the_run)
{
    in.read((char*)(&the_run.data_), sizeof(the_run.data_));
    return in;
}

std::ostream&
warthog::cpd::operator<<(std::ostream& out, warthog::cpd::rle_run32& the_run)
{
    out.write((char*)(&the_run.data_), sizeof(the_run.data_));
    return out;
}


