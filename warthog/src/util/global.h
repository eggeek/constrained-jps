#pragma once

#include <vector>
#include "constants.h"
#include "problem_instance.h"
#include "node_pool.h"
#include "gridmap.h"
#include "pqueue.h"
#include "search_node.h"
#include "solution.h"
using namespace std;
// set global variable that can be accessed everywhere
namespace global{

extern string alg;
extern warthog::mem::node_pool* nodepool;
extern warthog::solution* sol;

namespace statis {

  struct Log {
    uint32_t x, y, id, pid, padded_id, padded_pid, 
             sx, sy, tx, ty, sid, gid;
    string curalg, mapname;
    warthog::cost_t cg, opt_g;
    int subopt;

    string to_str() {
      string res = "";
      res = mapname + "," + curalg + "," +
        to_string(padded_id) + "," +
        to_string(id) + "," + 
        to_string(x) + "," +
        to_string(y) + "," +
        to_string(sid) + "," + 
        to_string(sx) + "," +
        to_string(sy) + "," +
        to_string(gid) + "," + 
        to_string(tx) + "," +
        to_string(ty) + "," +
        to_string(cg) + "," +
        to_string(opt_g) + "," +
        to_string(subopt);
      return res;
    }
  };

  extern vector<warthog::cost_t> dist;
  extern uint32_t subopt_expd;
  extern uint32_t subopt_gen;
  extern uint32_t subopt_insert;
  extern uint32_t scan_cnt;
  extern vector<Log> logs;

  extern uint32_t prunable;
  extern vector<Log> logs;

  Log gen(uint32_t id, warthog::cost_t gval, bool subopt);

  inline void init_dist(uint32_t size) {
    dist.resize(size);
    fill(dist.begin(), dist.end(), warthog::COST_MAX);
  }

  inline void update_subopt_insert(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) 
      subopt_insert++;
  }

  inline void update_gval(warthog::search_node* cur) {
    warthog::sn_id_t id = cur->get_id();
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && dist[id] > cur->get_g()) {
      dist[id] = cur->get_g();
    }
  }

  inline void update_subopt_expd(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) {
      subopt_expd++;
      // logs.push_back(gen(id, gval, 1));
    }
    // else {
    //   logs.push_back(gen(id, gval, 0));
    // }
  }

  inline void update_pruneable(warthog::search_node* cur) {
    warthog::sn_id_t pid = cur->get_parent();
    warthog::search_node* pa = pid == warthog::NO_PARENT? 
      nullptr: nodepool->get_ptr(pid);
    // parent is subopt
    if (!dist.empty() && pa != nullptr
        && pa->get_g() > dist[pa->get_id()]) 
      prunable++;
  }

  inline void update_subopt_touch(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) 
      subopt_gen++;
  }

  inline void clear() {
    subopt_insert = 0;
    subopt_expd = 0;
    subopt_gen = 0;
    scan_cnt = 0;
    prunable = 0;
  }

  inline void sanity_checking(uint32_t id, warthog::cost_t gval) {
    if (!dist.empty() && gval < dist[id]) {
      cerr << "invalid gval less than optimal: id=" << id << ", gval=" << gval << endl;
      assert(false);
      exit(1);
    }
  }

  inline void write_log(string fname) {
    std::ofstream fout(fname);
    string header = "map,alg,padded_id,id,x,y,sid,sx,sy,gid,tx,ty,curg,optg,subopt";
    fout << header << endl;
    for (auto& log: logs) {
      fout << log.to_str() << endl;
    }
    fout.close();
  }
};

namespace query {
  extern uint32_t startid, goalid;
  extern warthog::cost_t cur_diag_gval;
  extern warthog::gridmap *map;
extern warthog::problem_instance* pi;
extern warthog::pqueue_min* open;
  extern uint32_t jump_step;

  inline warthog::cost_t gval(uint32_t id) {
    warthog::cost_t res = warthog::INFTY;
    warthog::search_node* s = nodepool->get_ptr(id);
    if (s != nullptr && s->get_search_number() == pi->instance_id_) 
      res = min(res, s->get_g());
    return res;
  }

  // set gvalue on corner point
  inline void set_corner_gv(uint32_t id, warthog::cost_t g) {
    warthog::search_node* n = nodepool->get_ptr(id);
    if (n == nullptr) {
      n = nodepool->generate(id);
      n->init(pi->instance_id_, warthog::SN_ID_MAX, warthog::INFTY, warthog::INFTY);
      n->set_g(g);
    }
    else if (n->get_search_number() != pi->instance_id_) {
      n->init(pi->instance_id_, warthog::SN_ID_MAX, warthog::INFTY, warthog::INFTY);
      n->set_g(g);
    }
    else if (open->contains(n)) {
      // n has been generated and pushed in queue
      if (g < n->get_g()) {
        // and the current g is better, so n can be pruned.
        // n->relax(g, nullptr);
        n->set_expanded(true);
        // open->decrease_key(n); // this is to maintain the consistant heuristic
        // later when n is popped out, if g > 0 and parent is null,
        // we won't expand this node.
      }
    }
    else if (g < n->get_g()) {
      // n has been generated but not pushed yet,
      // implies that n is a corner point of another parent
      n->set_g(g);
      n->set_parent(warthog::NO_PARENT);
    }
  }
  inline void clear() {
    map = nullptr;
  }
};

  inline void clear() {
    statis::clear();
    query::clear();
  }
}
