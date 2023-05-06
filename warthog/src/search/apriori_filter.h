#ifndef WARTHOG_APRIORI_FILTER_H
#define WARTHOG_APRIORI_FILTER_H

// search/apriori_filter.h
//
// Sometimes during search it is desriable to ignore certain nodes that
// are identified apriori as not useful for any query. This filter helps to 
// achieve the goal by keeping track of a byte-sized label  for each node in a
// discrete graph. During search the filter can be queried about the 
// state of each node and those nodes whose bit is set can be ignored 
// (not generated or expanded).
//
// NB: This class duplicates the functionality of warthog::bitfifield_filter,
// trading space for a little bit of extra speed (no boolean logic ops).
// 
// 
// @author: dharabor
// @created: 2016-07-19

#include "constants.h"
#include "forward.h"
#include <cstdint>
#include <cstring>

namespace warthog
{

class apriori_filter 
{
    public:
        apriori_filter(warthog::graph::xy_graph* g);
        virtual ~apriori_filter();

        // returns true if the successor node specified by @param edge_idx
        // is being filtered. if the successor is not filtered, returns false
        bool
        filter(uint32_t node_id, uint32_t edge_idx);

        void
        set_flag_true(uint32_t node_id);

        void
        set_flag_false(uint32_t node_id);

        bool
        get_flag(uint32_t node_id);

        // clear all filter flags
        void
        reset_filter();

        // not used by this filter
        inline void
        set_target(uint32_t) { }

        inline size_t
        mem()
        {
            return 
                sizeof(*filter_)*filter_sz_ +
                sizeof(this);
        }

    private:
        warthog::graph::xy_graph* g_;
        uint8_t* filter_;
        uint32_t filter_sz_;
};

}

#endif
