#include "constants.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jps2_expansion_policy_prune2.h"
#include "octile_heuristic.h"
#include "zero_heuristic.h"
#include "scenario_manager.h"
#include "timer.h"
#include "global.h"

#include <stdlib.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <map>

using namespace std;
namespace w = warthog;
namespace G = global;
const double EPS = 1e-6;

struct ExpData {
  long long exp, gen, scan, pruneable;
  // count subopt gval:
  long long subopt_expd, subopt_gen;
  double time;
  void reset() {
    exp = gen = scan = pruneable = 0;
    subopt_expd = subopt_gen = 0;
  }
  void update(const w::solution& si) {
    exp += si.nodes_expanded_;
    // gen += si.nodes_touched_;
    gen += si.nodes_inserted_;
    time += si.time_elapsed_nano_;
    scan += G::statis::scan_cnt;
  }

  void update_subopt() {
    subopt_expd += G::statis::subopt_expd;
    // subopt_gen += G::statis::subopt_gen;
    subopt_gen += G::statis::subopt_insert;
    pruneable += G::statis::prunable;
  }

  string str() {
    string res = "";
    res += to_string(exp) + "\t" + 
           to_string(gen) + "\t" + 
           to_string((long long)time) + "\t" + to_string(scan);
    return res;
  }

  string subopt_str() {
    string res = "";
    res += to_string(subopt_gen) + "\t" +
           to_string(gen) + "\t" +
           to_string(subopt_expd) + "\t" +
           to_string(pruneable) + "\t" + 
           to_string(exp) + "\t" +
           to_string(scan);
    return res;
  }

  bool valid() {
    if (subopt_expd > exp ||
        subopt_gen > gen ||
        pruneable > subopt_gen) return false;
    return true;
  }
};
string mfile, sfile, type;
bool verbose = false;

void subcnt() {
  w::gridmap map(mfile.c_str());
  w::scenario_manager scenmgr;
  scenmgr.load_scenario(sfile.c_str());
  w::octile_heuristic heur(map.width(), map.height());
  w::zero_heuristic zheur;
  w::pqueue_min open;
  w::gridmap_expansion_policy expd_g(&map);
  w::jps2_expansion_policy expd_jps2(&map);
  w::jps2_expansion_policy_prune2 expd_cjps2(&map);


  w::flexible_astar<
    w::zero_heuristic, 
    w::gridmap_expansion_policy, 
    w::pqueue_min> dij(&zheur, &expd_g, &open);

  w::flexible_astar<
    w::octile_heuristic, 
    w::jps2_expansion_policy, 
    w::pqueue_min> jps2(&heur, &expd_jps2, &open);

  w::flexible_astar<
    w::octile_heuristic, 
    w::jps2_expansion_policy_prune2, 
    w::pqueue_min> cjps2(&heur, &expd_cjps2, &open);

  ExpData cnt_jps2, cnt_cjps2;
  ExpData* cnts[] = {&cnt_jps2, &cnt_cjps2};
  for (auto& it: cnts) it->reset();

  string header = "map\tid\tsubopt_gen\ttot_gen\tsubopt_expd\tpruneable\ttot_expd\tscnt\talg";
  cout << header << endl;
  int fromidx = max((int)scenmgr.num_experiments() - 100, 0);
  int toindx = (int)scenmgr.num_experiments();
  // int fromidx = 1960;
  // int toindx = 1961;

  G::query::map = &map;
  G::query::open = &open;
  for (int i=fromidx; i<toindx; i++) {
    w::experiment* exp = scenmgr.get_experiment(i);
    uint32_t sid = exp->starty() * exp->mapwidth() + exp->startx();
    uint32_t tid = exp->goaly() * exp->mapwidth() + exp->goalx();
    // cerr << i << " " << exp->startx() << " " << exp->starty() << " " << exp->distance() << endl;
    warthog::problem_instance pi_jps2(sid, tid, verbose);
    warthog::problem_instance pi_cjps2(sid, tid, verbose);
    warthog::problem_instance pi_dij(sid, w::SN_ID_MAX, verbose);
    warthog::solution sol_dij, sol_jps, sol_cjps;
    G::statis::clear();
    G::statis::init_dist(map.width() * map.height());
    G::sol = &sol_dij;
    dij.get_path(pi_dij, sol_dij);

    G::statis::clear();
    G::nodepool = expd_jps2.get_nodepool();
    G::sol = &sol_jps;
    jps2.get_path(pi_jps2, sol_jps);
    cnt_jps2.update(sol_jps);
    cnt_jps2.update_subopt();

    G::statis::clear();
    G::nodepool = expd_cjps2.get_nodepool();
    G::sol = &sol_cjps;
    cjps2.get_path(pi_cjps2, sol_cjps);
    cnt_cjps2.update(sol_cjps);
    cnt_cjps2.update_subopt();

    cout << mfile << "\t" << i << "\t" << cnt_jps2.subopt_str() << "\tjps2" << endl;
    cout << mfile << "\t" << i << "\t" << cnt_cjps2.subopt_str() << "\tc2jps2" << endl;

    if ((!cnt_cjps2.valid()) || (!cnt_jps2.valid())) {
      cerr << i << " " << exp->startx() << " " << exp->starty() << " " << exp->distance() << endl;
      exit(0);
    }
    for (auto &i: cnts) i->reset();
    assert(map.get_label(map.to_padded_id(sid)) != 0);
    assert(map.get_label(map.to_padded_id(tid)) != 0);
  }
  G::query::clear();
}

int main(int argc, char** argv) {
  // run expr and report suboptimal: ./expriment <map> <scen> subcnt
  // [suboptimal] step: for each query, run dijkstra first, 
  // then run jps2 and cjps2 and count the number of suboptimal node expansion/generation
  mfile = string(argv[1]);
  sfile = string(argv[2]);
  type = string(argv[3]);
  if (type == "subcnt") {
    subcnt();
  }
}
