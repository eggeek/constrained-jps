#ifndef __JSON_CONFIG_H_
#define __JSON_CONFIG_H_

#ifndef NDEBUG
#define VERBOSE true
#else
#define VERBOSE false
#endif

#include "constants.h"
#include <json.hpp>
#include <omp.h>

//
// - Definitions
//
typedef std::tuple<unsigned int, // Nodes expanded
                   unsigned int, // Nodes inserted
                   unsigned int, // Nodes touched
                   unsigned int, // Nodes updated
                   unsigned int, // Surplus nodes
                   unsigned int, // Total length of paths
                   double,       // Difference between perturbed and opti
                   double,       // Difference between free flow and opti
                   double,       // Difference between free flow and perturbed
                   double,       // Time spent in A*
                   double>       // Time to do the search
    t_results;

typedef struct config
{
    double hscale = 1.0;                // Modifier for heuristic's value
    double fscale = 0.0;                // Quality tolerance
    double time = DBL_MAX;
    uint32_t itrs = warthog::INF32;
    uint32_t k_moves = warthog::INF32;
    unsigned char threads = 0;
    bool verbose = VERBOSE;
    bool debug = false;
    bool thread_alloc = false;
    bool no_cache = false;
} config;

void
to_json(nlohmann::json& j, const config& c);

void
from_json(const nlohmann::json& j, config &c);

std::ostream&
operator<<(std::ostream& os, config &c);

std::istream&
operator>>(std::istream& is, config &c);

void
sanitise_conf(config& conf);

#endif // __JSON_CONFIG_H_
