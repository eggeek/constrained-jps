#ifndef WARTHOG_JPST_GRIDMAP_H
#define WARTHOG_JPST_GRIDMAP_H

// jpst_gridmap.h
// 
// Similar to warthog::jpst_gridmap but with some additional
// and redundant warthog::gridmap data to facilitate 
// Temporal Jump Point Search. 
// 
// In specific terms, we store a warthog::gridmap object which
// records whether a given xy has any temporal obstacles associated
// with it. This data helps us quickly scan the grid, in the manner
// of JPS, in order to identify temporal jump points.
//
// In addition, we store a second copy of the same gridmap but
// rotated 90 degrees. This speeds up grid scans moving vertically.
//
// @author: dharabor
// @created: 2019-11-11
//

#include "domains/gridmap.h"
#include "mapf/cbs.h"
#include "sipp/sipp_gridmap.h"

#include <vector>

namespace warthog
{

class jpst_gridmap
{
    public:
        jpst_gridmap(warthog::gridmap* gm);

        ~jpst_gridmap();

        // add a temporal obstacle at location (@param x, @param y). 
        // after the call, the location is blocked for the duration of the
        // **OPEN** interval (@param start_time, @param end_time).
        // 
        // NB: runs in time _linear_ to the number of intervals at (x, y)
        inline void
        add_obstacle(uint32_t x, uint32_t y, cost_t start_time, cost_t end_time, 
                     warthog::cbs::move action = warthog::cbs::move::WAIT)
        {
            sipp_map_->add_obstacle(x, y, start_time, end_time, action);

            // record the fact that there are temporal obstacles at this location
            uint32_t node_id = y * t_gm_->header_width() + x;
            uint32_t gm_id = t_gm_->to_padded_id(node_id);
            t_gm_->set_label(gm_id, true);
        }


        // remove all temporal obstacles at location (@param x, @param y)
        // after the call this location has a single safe interval. 
        // for traversable tile, this interval begins
        // at time 0 and finishes at time INF.
        // for obstacle tiles, this interval begins and ends at time
        // COST_MAX (i.e. infinity)
        inline void
        clear_obstacles(uint32_t x, uint32_t y)
        {
            sipp_map_->clear_obstacles(x, y);

            // record the fact that there are no temporal obstacles at this location
            uint32_t node_id = y * t_gm_->header_width() + x;
            uint32_t gm_id = t_gm_->to_padded_id(node_id);
            t_gm_->set_label(gm_id, false);
        }

        // return a reference to the (@param index)th safe interval 
        // associated with the grid cell index @param node_id. 
        warthog::sipp::safe_interval&
        get_safe_interval(uint32_t node_id, uint32_t index)
        {
            return sipp_map_->get_safe_interval(node_id, index);
        }

        // return a reference to the list of safe intervals for the
        // grid cell with index @param node_id
        std::vector<warthog::sipp::safe_interval>& 
        get_all_intervals(uint32_t node_id)
        {
            return sipp_map_->get_all_intervals(node_id);
        }

        warthog::sipp::safe_interval* 
        find_first_reachable(uint32_t xy_id, warthog::cost_t current_time)
        {
            for(auto& si : intervals_.at(xy_id))
            {
                if(si.e_time_ >= current_time) 
                { return &si; }
            }
            return 0;
        }

        size_t
        mem()
        {
            size_t retval = 
            sipp_map_->mem() + 
            t_gm_->mem();
            retval += sizeof(this);

            return retval;
        }

        // track which xy locations are static obstacles
        warthog::gridmap* gm_; 

        // track which xy locations have temporal obstacles
        warthog::gridmap* t_gm_;

        // track when temporal obstacles appear and disappear
        warthog::sipp_gridmap* sipp_map_;

    private:
        std::vector<std::vector<warthog::sipp::safe_interval>> intervals_;
};

}

#endif
