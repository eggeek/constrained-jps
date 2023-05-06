#include "workload_manager.h"

warthog::util::workload_manager::workload_manager(size_t num_elements) 
{
    filter_sz_ = (num_elements >> warthog::LOG2_DBWORD_BITS)+1;
    filter_ = new warthog::dbword[filter_sz_];
    set_all_flags(false);
}

warthog::util::workload_manager::~workload_manager()
{
    delete [] filter_;
}

void
warthog::util::workload_manager::set_all_flags(bool val)
{
    warthog::dbword w_val = (warthog::dbword)(val ? ~0 : 0);
    for(size_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = w_val;
    }
}

void 
warthog::util::workload_manager::set_flag(uint32_t node_id, bool val)
{
    uint32_t index = node_id >> warthog::LOG2_DBWORD_BITS;
    uint32_t pos = node_id & DBWORD_BITS_MASK;

    if(index >= filter_sz_) { return; }
    if(val)
    {
        filter_[index] |= (1 << pos);
    }
    else
    {
        filter_[index] &= ~(1 << pos);
    }
}

bool
warthog::util::workload_manager::get_flag(uint32_t id) 
{
   assert((id / warthog::DBWORD_BITS) < filter_sz_);
   uint32_t word = id / warthog::DBWORD_BITS;
   uint32_t pos = id % warthog::DBWORD_BITS;
   if(word >= filter_sz_) { return false; }
   return this->filter_[word] & (1 << pos);
}

uint32_t
warthog::util::workload_manager::num_flags_set()
{
    uint32_t count = 0;
    for(size_t i = 0; i < filter_sz_*sizeof(warthog::dbword); i++)
    {
        count += (uint32_t)__builtin_popcount(filter_[i]);
    }
    return count;
}

void
warthog::util::workload_manager::set_all_flags_complement()
{
    for(size_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = ~filter_[i];
    }
}

