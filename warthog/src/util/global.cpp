#include "global.h"
#include "constants.h"
#include "node_pool.h"
#include "pqueue.h"
using namespace global;
uint32_t statis::subopt_expd = 0;
uint32_t statis::subopt_gen = 0;
uint32_t statis::scan_cnt = 0;
uint32_t statis::subopt_insert = 0;
string global::alg = "";
uint32_t statis::prunable = 0;
vector<warthog::cost_t> statis::dist = vector<warthog::cost_t>();
vector<statis::Log> statis::logs = vector<statis::Log>();
warthog::problem_instance* query::pi = nullptr;
warthog::mem::node_pool* global::nodepool = nullptr;
uint32_t query::startid = warthog::INF32;
uint32_t query::goalid = warthog::INF32;
warthog::cost_t query::cur_diag_gval = warthog::INFTY;
warthog::gridmap* query::map = nullptr;
warthog::pqueue_min* query::open = nullptr;
uint32_t global::query::jump_step = 0;
warthog::solution* global::sol = nullptr;

global::statis::Log global::statis::gen(uint32_t id, warthog::cost_t gval, bool subopt) {
  global::statis::Log c;
  c.cg = gval;
  c.opt_g = id < dist.size()? dist[id]: 0;
  c.curalg = global::alg;
  c.mapname = string(global::query::map->filename());
  global::query::map->to_unpadded_xy(id, c.x, c.y);
  global::query::map->to_unpadded_xy(global::query::startid, c.sx, c.sy);
  global::query::map->to_unpadded_xy(global::query::goalid, c.tx, c.ty);
  c.padded_id = id;
  c.id = global::query::map->to_padded_id(id);
  c.sid = global::query::map->to_padded_id(global::query::startid);
  c.gid = global::query::map->to_padded_id(global::query::goalid);
  c.subopt = subopt;
  return c;
}
