#pragma once
#include <cassert>
#include "constants.h"
#include "gridmap.h"
#include "global.h"

using namespace std;

namespace warthog {

/*
 *
 * a    b
 * +----+....
 * .\...|....
 * ..\..|....
 * ...\.|....
 * ....\|....
 * .....+....
 *
 */

struct Constraint2 {
  cost_t ga, gb,  // gvalue of node a and b
         dC;      // |ab|
                  // when jumpcost >= t, the jump point is better reached from b
  int i,          // i-th diag step since the constraint is activated
      d,          // |ab|
      L,          // the constraint is no longer applicable when i>=L
      ti;         // terminate when i=ti, all cardinal nodes are better reached from b
  jps::direction dir, pdir; // direction and perpendicular direction
  inline int jlimt() const { return d - i; }

  /*
   * when i=ti, we should terminate the diagonal expansion
   * thus, ti s.t. ga+ti*sqrt(2) > gb + octile(b, m) +1
   * a-----b
   *  \
   *   \m
   *    + ti
   * octile(b, m) exists since all nodes above ti are scanned.
   */
  inline void calc_ti() {
    // case 1: ti-1 <= d-ti
    if ((gb + dC + 2) < dC + 1 + ga + DBL_ROOT_TWO) {
      if (ga + DBL_ROOT_TWO <= gb + dC + 2) {
        ti = ceil((gb + dC + 2 - (ga + DBL_ROOT_TWO)) / 2.0);
      }
      else {
        ti = 0;
      }
    }
    else {
      // case 2: ti-1 >= d-ti
      static const cost_t delta = DBL_ROOT_TWO - 1;
      ti = ceil((gb - ga + delta * d) / (delta*2));
      ti = max(ti, (d+2)/2);
    }
  }
  // make s step from ai, check whether it is better reached from b 
  inline bool better_from_b(int s) const {
    assert(s <= jlimt());
    int d1 = d - i - s;
    int d2 = i - 1;
    int dia = min(d1, d2);
    cost_t from_b = gb + dia * DBL_ROOT_TWO + (d1 + d2 - (dia<<1) + 1);
    return from_b <= ga + i*DBL_ROOT_TWO + s; 
  }

  // a is better reached from b, terminate when i=0
  inline bool dominated() const { return i>=0 && ga >= gb + dC; }
  inline bool next() {
    if (i < 0) return true; // no applicable constraint
    // next diagonal move is better reached from b
    if (++i >= ti) return false;
    // next diag move:
    // 1. passed the intersection;
    // 2. cardinal move until the jump limit is better reached from a;
    if (i >= d || i >= L) {
      deactivate();
      return true;
    }
    return true;
  }

  // rest the constraint 
  inline void reset() { 
    // lastDiag = 0;
    i = -1;
    d = L = MAXSIDE;
    ga = dC = 0;
    gb = MAXSIDE;
    dC = MAXSIDE;
    dir = jps::NONE;
    pdir = jps::NONE;
  }

  // this happen when it becomes invalid at i-th step, 
  // but keep moving diagonal;
  // dont change the maxDiag, lastDiag
  inline void deactivate() {
    i = -1;
    d = L = MAXSIDE;
  }

  // call this function before each diagonal move
  inline void init_before_diag(jps::direction dir_, jps::direction pdir_) {
    dir = dir_, pdir = pdir_;
  }
};

class online_jps_pruner2 {
public:
  Constraint2 north, south, east, west, h, v;
  void setup(Constraint2& c, cost_t ga, cost_t gb, cost_t jumpcost) {
    // if the new constraint applicable, update, 
    // otherwise deactivate
    if (ga + jumpcost > gb) {
      c.ga = ga, c.gb = gb, c.dC = jumpcost; 
      c.d = jumpcost;
      c.i = 0;
      c.calc_ti();
      static const cost_t div = 2.0 - DBL_ROOT_TWO;
      // L = floor((ga+d-gb)/(2-sqrt(2)))
      c.L = floor((c.ga + c.dC - c.gb) / div);
    }
    else c.deactivate();
  }
  bool t_labelv, t_labelh; // store the original label before setting artificial obstacle
  int t_mapid, t_rmapid;
  uint32_t jump_step;      // the step of previous cardinal scanning
  cost_t jumpcost;         // the cost (step) of previous scanning

  void reset_constraints() {
    north.reset();
    south.reset();
    east.reset();
    west.reset();
    v.reset();
    h.reset();
  }

  /*
   *        ---------+b
   *        |        | ^
   *        |        | lv
   * ai     m<- lh ->| v
   * +<-d'->+--------+
   *        b'
   * at i-th step, we reach a dead-end b' before hitting jump limit
   * then we try to build a constraint on b' based on previous constraint on b
   * in this stage, there is an obstacle-free path from b to m
   * the new gb: gb + |<b, m>| plus one cardinal move
   * the new d:  number of cardinal step from ai to b'
   *
   */
  inline void update_constraint(Constraint2& c, int dx, int dy, cost_t ai2b_, cost_t known_gb) {
    int l = min(dx, dy);
    cost_t dist = l * DBL_ROOT_TWO + (cost_t)(dx + dy - (l<<1));
    cost_t new_ga = c.ga + c.i * DBL_ROOT_TWO;
    cost_t new_gb = min(known_gb, c.gb + dist + 1);
    setup(c, new_ga, new_gb, ai2b_);
  }

  /*
   * before scan: if the constraint is active, set temp obstacle based on jlimt
   */
  inline void before_scanv(gridmap* rmap, int rmapid, int direct) {
    if (v.i>0) {
      t_rmapid = rmapid + direct * (v.jlimt() + 1);
      assert(t_rmapid >= 0);
      t_labelv = rmap->get_label(t_rmapid);
      rmap->set_label(t_rmapid, false);
    }
  }

  inline void before_scanh(gridmap* map, int mapid, int direct) {
    if (h.i>0){
      t_mapid = mapid + direct * (h.jlimt() + 1);
      assert(t_mapid >= 0);
      t_labelh = map->get_label(t_mapid);
      map->set_label(t_mapid, false);
    }
  }

  /*
   * after scan:
   * 1. if the constraint is active: 
   *  1.1 unset the temp obstacle;
   *  1.2 try to resue the current constraint if we stop before hitting jlimit;
   * 2. update the constraint if we can have a stronger bound on node_id,
   *   e.g. it has a smaller gvalue due to the previous expansion;
   * return true if continue, false terminate the expansion
   */
  inline bool after_scanv(gridmap* rmap, uint32_t node_id, 
      uint32_t &jpid, cost_t& cost) {
    if (v.i>0) { // the constraint is active
      rmap->set_label(t_rmapid, t_labelv); // 1.1
      if ((int)jump_step < v.jlimt()) {
        if (v.better_from_b(jump_step)) {
          int dy = v.i-1;
          int dx = v.d-v.i-jump_step;
          update_constraint(v, dx, dy, (cost_t)jump_step, global::query::gval(node_id));
          jpid = INF32;
          if (v.dominated()) return false;
        }
        else { // the constraint is no longer applicable
          v.deactivate();
          return true;
        }
      }
    }
    else { // 2
      cost_t gb = global::query::gval(node_id);
      if (global::query::cur_diag_gval+cost > gb) {
        setup(v, global::query::cur_diag_gval, gb, cost);
      }
    }
    return true;
  }

  inline bool after_scanh(gridmap* map, uint32_t node_id, 
      uint32_t &jpid, cost_t& cost) {
    if (h.i>0) {
      map->set_label(t_mapid, t_labelh);
      if ((int)jump_step < h.jlimt()) {
        if (h.better_from_b(jump_step)) {
          int dy = h.i-1;
          int dx = h.d-h.i-jump_step;
          update_constraint(h, dx, dy, (cost_t)jump_step, global::query::gval(node_id));
          jpid = INF32;
          if (h.dominated()) return false;
        }
        else {
          h.deactivate();
          return true;
        }
      }
    }
    else {
      cost_t gb = global::query::gval(node_id);
      if (global::query::cur_diag_gval+cost > gb) {
        setup(h, global::query::cur_diag_gval, gb, cost);
      }
    }
    return true;
  }
};
}
