#include "bitfield_filter.h"
#include "search_node.h"
#include <cassert>

warthog::bitfield_filter::bitfield_filter(size_t num_elements) 
{
    filter_sz_ = (num_elements >> warthog::LOG2_DBWORD_BITS)+1;
    filter_ = new warthog::dbword[filter_sz_];
    reset_filter();
}

warthog::bitfield_filter::~bitfield_filter()
{
    delete [] filter_;
}

void 
warthog::bitfield_filter::set_flag_true(sn_id_t node_id)
{
    size_t index = node_id >> warthog::LOG2_DBWORD_BITS;
    size_t pos = node_id & DBWORD_BITS_MASK;
    filter_[index] |= (1 << pos);
}

void
warthog::bitfield_filter::set_flag_false(sn_id_t node_id)
{
    size_t index = node_id >> warthog::LOG2_DBWORD_BITS;
    size_t pos = node_id & DBWORD_BITS_MASK;
    filter_[index] &= ~(1 << pos);
}

void 
warthog::bitfield_filter::reset_filter()
{
    for(size_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = 0;
    }
}

bool
warthog::bitfield_filter::filter(sn_id_t node_id, uint32_t edge_idx)
{
    return get_flag(node_id);
}

bool
warthog::bitfield_filter::get_flag(sn_id_t id) 
{
    size_t index = id >> warthog::LOG2_DBWORD_BITS;
    size_t pos = id & DBWORD_BITS_MASK;
    return filter_[index] & (1 << pos);
}

