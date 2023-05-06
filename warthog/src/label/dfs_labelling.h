#ifndef WARTHOG_DFS_LABELLING_H
#define WARTHOG_DFS_LABELLING_H

// label/dfs_labelling.h
//
// Collects a variety of different labels that we can compute for
// the down-closure of a node. Intended for use with forward-driven
// contraction hierarchies.
//
// Current edge labels:
//  - id range: specifying the range of ids in the down closure of each edge
//
//  - bounding box: a rectangular bounding box that contains every
//  node in the down closure of each edge
//
// @author: dharabor
// @created: 2017-12-06
//

#include "contraction/contraction.h"
#include "util/geom.h"
#include "util/timer.h"
#include "sys/forward.h"

#include <cassert>
#include <fstream>
#include <vector>
#include <cstdint>

namespace warthog
{

namespace label
{

struct dfs_label
{
    typedef uint8_t T_FLAG;
    dfs_label() { }

    dfs_label&
    operator=(const dfs_label& other)
    {
        bbox_ = other.bbox_;
        return *this;
    }

    void
    merge(const dfs_label& other)
    {
        bbox_.grow(other.bbox_);
    }

    void
    print(std::ostream& out)
    {
        out << " dfs_label";
        bbox_.print(out);
    }
    warthog::geom::rectangle bbox_;
};

std::istream&
operator>>(std::istream& in, warthog::label::dfs_label& label);

std::ostream&
operator<<(std::ostream& out, warthog::label::dfs_label& label);

class dfs_labelling
{
    friend std::ostream&
    operator<<(std::ostream& out, dfs_labelling& lab);

    friend std::istream&
    operator>>(std::istream& in, warthog::label::dfs_labelling& lab);

    public:

        dfs_labelling(warthog::ch::ch_data*);

        ~dfs_labelling();

        inline warthog::ch::ch_data*
        get_ch_data()
        {
            return chd_;
        }

        dfs_label&
        get_label(uint32_t node_id, uint32_t edge_idx)
        {
            assert(edge_idx < lab_->at(node_id).size());
            return lab_->at(node_id).at(edge_idx);
        }

        inline uint32_t
        get_dfs_index(uint32_t graph_id) { return dfs_order_->at(graph_id); }

        inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            for(uint32_t i = 0; i < lab_->size(); i++)
            {
                retval += (sizeof(dfs_label) )
                    * lab_->at(i).size();
            }
            retval += sizeof(int32_t) * dfs_order_->size();
            return retval;
        }

        // compute labels for all nodes specified by the given workload
        void
        precompute(warthog::util::workload_manager* workload);

    private:

        warthog::ch::ch_data* chd_;

        // these two pointers are populated from ::chd_
        warthog::graph::xy_graph* g_;
        std::vector<uint32_t>* level_;

        uint32_t apex_id_;

        std::vector<uint32_t>* dfs_order_;
        std::vector< std::vector< dfs_label >>* lab_;

        // Computes a DFS post-order id for every node in a contraction
        // hierarchy (i.e. a top-down traversal)
        // @param id of the highest node in the contraction hierarchy
        uint32_t
        compute_dfs_postorder_ids_ch(std::vector<uint32_t>* dfs_ids);

        // Perform a DFS traversal through the contraction hierarchy
        // and compute a bounding box for each up and down edge.
        // For down edges, the box contains all nodes in the down closure.
        // For up edges, the box contains all nodes in the up closure C
        // PLUS for every node n \in C, the set of nodes found in the down
        // closure for n.
        //
        // @param workload specifies which nodes to
        void
        compute_dfs_labels(warthog::util::workload_manager* workload);
};

std::istream&
operator>>(std::istream& in, warthog::label::dfs_labelling& lab);

std::ostream&
operator<<(std::ostream& in, warthog::label::dfs_labelling& lab);

}


}

#endif
