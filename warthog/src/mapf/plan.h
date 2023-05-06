#ifndef WARTHOG_MAPF_PLAN_H
#define WARTHOG_MAPF_PLAN_H

// mapf/plan.h
// 
// Describes a coordinated plan to move a set of agents, 
// each from its initial position and to a designated
// target location, all while avoiding collisions with 
// other agents.
//
// @author: dharabor
// @created: 2019-11-02
//

#include "sys/constants.h"
#include "search/solution.h"

#include <vector>
#include <iostream>

namespace warthog
{
namespace mapf
{

struct plan
{
    // clear existing plans for agents. individual plans can
    // be specified by passing @param agent_index.
    // the default value is UINT32_MAX, which clears
    // the plans of **ALL** agents. 
    void
    clear(uint32_t agent_index = UINT32_MAX)
    {
        if(agent_index < paths_.size())
        {
            paths_.at(agent_index).reset(); 
            return;
        }

        if(agent_index == UINT32_MAX) 
        { paths_.clear(); }
    }

    std::vector<warthog::solution> paths_;
};

// load an existing plan
std::istream&
operator>>(std::istream& in, warthog::mapf::plan& theplan);

// write out the current plan
std::ostream&
operator<<(std::ostream& out, warthog::mapf::plan& theplan);

}

}

#endif
