#ifndef WARTHOG_WORKLOAD_MANAGER_H
#define WARTHOG_WORKLOAD_MANAGER_H

// util/workload_manager.h
//
// Pre-processing involves applying some operation to the nodes of an input 
// graph and recording the result in an auxilliary data structure.
// Usually preprocessing is  performed on all the nodes of the graph but 
// sometimes it is useful to only target a subset.
// A workload_manager is used to specify the scope of precomputation work
// by collecting the ids of all nodes intended for preprocessing.
//
// @author: dharabor
// @created: 2017-09-16

#include "constants.h"

#include <cassert>
#include <cstdint>
#include <cstring>

namespace warthog
{

namespace util
{

class workload_manager
{
    public:
        
        // on construction, the workload manager is empty 
        // (i.e. the "work" flag for every node is set to false)
        workload_manager(size_t num_nodes);
        virtual ~workload_manager();

        bool
        get_flag(uint32_t node_id);

        // set a single flag
        // (i.e. add one node to the precomputation workload)
        void
        set_flag(uint32_t node_id, bool val);

        // set all flags to the same as @bool val
        // (i.e. add all nodes to the precomputation workload)
        void
        set_all_flags(bool val);

        // flip every bit to its opposite value
        // (i.e. take the complement of the set of nodes in the workload)
        void
        set_all_flags_complement(); 

        // return the number of nodes whose flag is set
        // (i.e. determine the size of the precomputation workload)
        uint32_t
        num_flags_set();

        inline size_t
        mem()
        {
            return 
                sizeof(*filter_)*filter_sz_ +
                sizeof(this);
        }

    private:
        warthog::dbword* filter_;
        size_t filter_sz_;
};

}

}

#endif
