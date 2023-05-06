#include "cbs_ll_heuristic.h"
#include "dummy_listener.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "labelled_gridmap.h"
#include "solution.h"
#include "vl_gridmap_expansion_policy.h"
#include "zero_heuristic.h"

#include <functional>
#include <stdint.h>

warthog::cbs_ll_heuristic::cbs_ll_heuristic(warthog::gridmap* gm)
{
    sol_ = new warthog::solution();
    open_ = new warthog::pqueue_min();
    zh_ = new warthog::zero_heuristic();
    listener_ = new warthog::cbs_ll_heuristic::listener(this);
    gm_sz_ = gm->width() * gm->height();

    bool manhattan = true;
    expander_ = new warthog::gridmap_expansion_policy(gm, manhattan);

    alg_ = new warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::gridmap_expansion_policy,
        warthog::pqueue_min, 
        warthog::cbs_ll_heuristic::listener > 
            (zh_, expander_, open_, listener_);
}

warthog::cbs_ll_heuristic::~cbs_ll_heuristic() 
{
    delete alg_;
    delete expander_;
    delete listener_;
    delete zh_;
    delete open_;
    delete sol_;
}

void
warthog::cbs_ll_heuristic::set_current_target(warthog::sn_id_t target_id)
{
    uint32_t target_xy_id = (uint32_t)target_id;
    std::unordered_map<uint32_t, uint32_t>::iterator it = t_map_.find(target_xy_id);

    // already have precomputed h-values for this target
    if(it != t_map_.end()) 
    {
        t_index_ = it->second;
        return;
    }

    // new target; precompute h-values from scratch
    t_index_ = (uint32_t)t_map_.size();
    t_map_[(uint32_t)target_id] = t_index_;
    h_.push_back(std::vector<warthog::cost_t>());
    h_[t_index_].resize(gm_sz_, warthog::INF32);

    sol_->reset();
    warthog::problem_instance problem(target_id);
    alg_->get_pathcost(problem, *sol_);
}

size_t
warthog::cbs_ll_heuristic::mem() 
{ 
    size_t sz = sizeof(this);
    for(uint32_t i = 0; i < h_.size(); i++)
    {
        sz += sizeof(warthog::cost_t) * h_[i].size();
    }
    sz += sizeof(void*)*h_.size();
    sz += sizeof(uint32_t)*2*t_map_.size();
    sz += open_->mem();
    sz += expander_->mem();
    sz += alg_->mem();
    return sz;
}
