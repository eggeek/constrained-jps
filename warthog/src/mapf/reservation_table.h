#ifndef WARTHOG_RESERVATION_TABLE_H
#define WARTHOG_RESERVATION_TABLE_H

// mapf/reservation_table.h
//
// A reservation table that describes a solution to a
// MAPF problem. Each cell that appears on the path
// of an agent is "marked" in the reservation table.
// The idea is to share this information among agents so
// they can plan without colliding into one another.
//
// This implementation uses a single bit to represent each
// cell in a time-expanded grid graph.
//
// For more details see: 
// Sharon, Guni, et al. "Conflict-based search for optimal multi-agent pathfinding." 
// Artificial Intelligence 219 (2015): 40-66.
//
// @author: dharabor
// @created: 2018-11-05
//

#include "constants.h"
#include "cpool.h"

#include <stdint.h>
#include <vector>

namespace warthog
{

class reservation_table
{
    static const uint32_t LOG2_QWORD_SZ = 6;
    public:
        reservation_table(uint32_t map_sz) : map_sz_(map_sz) 
        {
            map_sz_in_qwords_ = (map_sz_ >> LOG2_QWORD_SZ)+1;
            pool_ = new warthog::mem::cpool(map_sz_in_qwords_);
        }
        ~reservation_table() 
        {
            delete pool_;
        }

        inline bool
        is_reserved(uint32_t xy_id, uint32_t timestep)
        {
            if(timestep >= table_.size()) { return false; }
            return table_[timestep][xy_id >> LOG2_QWORD_SZ] & 
                   (1 << (xy_id & 63));
        }

        inline bool
        is_reserved(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = time_indexed_map_id >> 32;
            uint32_t xy_id = time_indexed_map_id & UINT32_MAX;
            return is_reserved(xy_id, timestep);
        }

        inline void
        reserve(uint32_t xy_id, uint32_t timestep)
        {
            assert(xy_id < map_sz_);
            while(table_.size() <= timestep)
            {
                uint64_t* map = (uint64_t*)pool_->allocate();
                for(uint32_t i = 0; i < map_sz_in_qwords_; i++)
                { map[i] = 0; }
                table_.push_back(map);
            }
            table_[timestep][xy_id >> LOG2_QWORD_SZ] |= (1 << (xy_id & 63));
        }

        inline void
        reserve(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = time_indexed_map_id >> 32;
            uint32_t xy_id = time_indexed_map_id & UINT32_MAX;
            reserve(xy_id, timestep);
        }

        inline void
        unreserve(uint32_t xy_id, uint32_t timestep)
        {
            assert(timestep < table_.size());
            assert(xy_id < map_sz_);
            table_[timestep][xy_id >> LOG2_QWORD_SZ] &= ~(1 << (xy_id & 63));
        }

        inline void
        unreserve(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = (uint32_t)(time_indexed_map_id >> 32);
            uint32_t xy_id = (uint32_t)(time_indexed_map_id & UINT32_MAX);
            unreserve(xy_id, timestep);
        }

        inline void
        clear_reservations()
        {
            for(uint64_t* map : table_)
            {
                for(uint32_t i = 0; i < map_sz_in_qwords_; i++)
                { map[i] = 0; }
            }
        }

        size_t
        mem()
        {
            size_t retval = sizeof(this);
            retval += pool_->mem();
            retval += sizeof(uint64_t*)*table_.size();
            return retval;
        }

    private:
        std::vector<uint64_t*> table_;
        uint32_t map_sz_;
        uint32_t map_sz_in_qwords_;
        warthog::mem::cpool* pool_;


};

}

#endif
