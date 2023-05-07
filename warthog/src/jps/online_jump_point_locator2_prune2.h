#pragma once
// online_jump_point_locator2_prune2.h
// @author: shizhe 
// @created: 30/06/2021
//

#include "constants.h"
#include "gridmap.h"
#include "jps.h"
#include "online_jps_pruner2.h"
#include "node_pool.h"
#include "search_node.h"

//class warthog::gridmap;
namespace warthog
{

class online_jump_point_locator2_prune2
{
	public: 
		online_jump_point_locator2_prune2(gridmap* map, online_jps_pruner2* pruner);
		~online_jump_point_locator2_prune2();

		void
		jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
				std::vector<uint32_t>& jpoints,
				std::vector<warthog::cost_t>& costs);

		uint32_t 
		mem()
		{
			return sizeof(*this) + rmap_->mem();
		}
    online_jps_pruner2* jp;
    search_node* pa;

    inline warthog::gridmap* get_rmap() { return rmap_; }
    inline warthog::gridmap* get_map() { return map_; }

	private:
		void
		jump_north(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_south(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_east(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_west(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_northeast(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_northwest(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_southeast(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);
		void
		jump_southwest(
				std::vector<uint32_t>& jpoints, 
				std::vector<warthog::cost_t>& costs);

		// these versions can be passed a map parameter to
		// use when jumping. they allow switching between
		// map_ and rmap_ (a rotated counterpart).
		void
		__jump_north(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				warthog::gridmap* mymap);
		void
		__jump_south(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);
		void
		__jump_east(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);
		void
		__jump_west(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);

		// these versions perform a single diagonal jump, returning
		// the intermediate diagonal jump point and the straight 
		// jump points that caused the jumping process to stop
		void
		__jump_northeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_northwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_southeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_southwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);

		// functions to convert map indexes to rmap indexes
		inline uint32_t
		map_id_to_rmap_id(uint32_t mapid)
		{
			if(mapid == warthog::INF32) { return mapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			map_->to_unpadded_xy(mapid, x, y);
			ry = x;
			rx = map_->header_height() - y - 1;
			return rmap_->to_padded_id(rx, ry);
		}

		// convert rmap indexes to map indexes
		inline uint32_t
		rmap_id_to_map_id(uint32_t rmapid)
		{
			if(rmapid == warthog::INF32) { return rmapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			rmap_->to_unpadded_xy(rmapid, rx, ry);
			x = ry;
			y = rmap_->header_width() - rx - 1;
			return map_->to_padded_id(x, y);
		}

		warthog::gridmap*
		create_rmap();

		warthog::gridmap* map_;
		warthog::gridmap* rmap_;
		//uint32_t jumplimit_;

		uint32_t current_goal_id_;
		uint32_t current_rgoal_id_;
		uint32_t current_node_id_;
		uint32_t current_rnode_id_;
    vector<bool> iscorner;

    // nxtjp[d][id] stores next jump point in direction `d` (NSEW) at `id`
    // vector<pair<uint32_t, cost_t>> nxtjp[4];

    vector<uint32_t> rmap2mapid;

    public:
    inline void init_tables() {
      rmap2mapid.resize(rmap_->height()*rmap_->width());
      iscorner.resize(map_->height()*map_->width());
      fill(rmap2mapid.begin(), rmap2mapid.end(), -1);
      fill(iscorner.begin(), iscorner.end(), false);
      int mh = map_->header_height();
      int mw = map_->header_width();
      for (int x=0; x<mw; x++)
      for (int y=0; y<mh; y++) {
        uint32_t px, py, pid;// rpid;
        pid = map_->to_padded_id(y*mw+x);
        map_->to_padded_xy(pid, px, py);
        // rpid = map_id_to_rmap_id(pid);

        // assert(rpid < rmap2mapid.size());
        // rmap2mapid[rpid] = pid;
        assert(pid < iscorner.size());
        iscorner[pid] = map_->is_corner(px, py);
      }

      // // global::corner_gv.resize(map_->height()*map_->width());
      // // for (auto& it: global::corner_gv) {
      // //   it.g = INF, it.searchid = INF;
      // // }
      //
      // for (int i=0; i<4; i++) {
      //   nxtjp[i].resize(map_->height() * map_->width());
      //   for (auto& it: nxtjp[i]) it = {INF, max(map_->height(), map_->width()) * ONE};
      //   for (int x=0; x<mw; x++)
      //   for (int y=0; y<mh; y++) {
      //     int cx = x + dx[i], cy = y + dy[i];
      //     uint32_t pid;
      //     pid = map_->to_padded_id(y*mw+x);
      //     if (!iscorner[pid]) continue;
      //     cost_t pcost = 0;
      //     while (cx >= 0 && cx < mw && cy >= 0 && cy < mh) {
      //       uint32_t padded_x, padded_y, padded_id;
      //       pcost += ONE;
      //       padded_id = map_->to_padded_id(cy*mw+cx);
      //       map_->to_padded_xy(padded_id, padded_x, padded_y);
      //       if (iscorner[padded_id]) {
      //         nxtjp[i][pid] = {padded_id, pcost};
      //         break;
      //       }
      //       cx += dx[i], cy += dy[i];
      //     }
      //   }
      // }
    }

    // inline void _backwards_gval_update(uint32_t jpid, cost_t jpc, cost_t pgv, int dirid) {
    //   cost_t cur_cost = 0;
    //   while (cur_cost + nxtjp[dirid][jpid].second < jpc) {
    //     cur_cost += nxtjp[dirid][jpid].second;
    //     global::query::set_corner_gv(nxtjp[dirid][jpid].first, pgv+jpc-cur_cost);
    //     jpid = nxtjp[dirid][jpid].first;
    //   }
    // }

    inline void backwards_gval_update_NS(uint32_t jpid, uint32_t r_jpid, 
        cost_t jpc, cost_t pgv, jps::direction dir) {
      cost_t cur_cost = 0, nxt_cost;
      uint32_t nxtjp;
      while (true) {
        nxtjp = jpid;
        _backwards_gval_update_online_NS(nxtjp, r_jpid, nxt_cost, dir);
        if (cur_cost + nxt_cost >= jpc) break;
        cur_cost += nxt_cost;
        global::query::set_corner_gv(nxtjp, pgv+jpc-cur_cost);
        jpid = nxtjp;
      }
    }

    inline void backwards_gval_update_EW(uint32_t jpid,
        cost_t jpc, cost_t pgv, jps::direction dir) {
      cost_t cur_cost = 0, nxt_cost;
      int cnt = 0;
      uint32_t nxtjp;
      while (true) {
        nxtjp = jpid;
        _backwards_gval_update_online_EW(nxtjp, nxt_cost, dir);
        cnt++;
        if (cnt == 2 || cur_cost + nxt_cost >= jpc) break;
        cur_cost += nxt_cost;
        global::query::set_corner_gv(nxtjp, pgv+jpc-cur_cost);
        jpid = nxtjp;
      }
    }

    inline void _backwards_gval_update_online_NS(
        uint32_t& jpid, uint32_t& r_jpid, cost_t& jcost, jps::direction dir) {
      switch (dir) {
        uint32_t rid;
        case jps::NORTH: {
                           __jump_north(r_jpid, INF32, rid, jcost, rmap_);
                           jpid -= jp->jump_step * map_->width();
                           r_jpid = rid;
                           break;
                         }
        case jps::SOUTH: {
                           __jump_south(r_jpid, INF32, rid, jcost, rmap_);
                           jpid += jp->jump_step * map_->width();
                           r_jpid = rid;
                           break;
                         }
        default: break;
      }
    }

    inline void _backwards_gval_update_online_EW(
        uint32_t& jpid, cost_t& jcost, jps::direction dir) {

      uint32_t id;
      switch (dir) {
        case jps::EAST: {
                          __jump_east(jpid, INF32, id, jcost, map_);
                          jpid += jp->jump_step;
                          break;
                        }
        case jps::WEST: {
                          __jump_west(jpid, INF32, id, jcost, map_);
                          jpid -= jp->jump_step;
                          break;
                        }
        default: break;
      }
    }
};

}
