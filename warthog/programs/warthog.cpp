// warthog.cpp
//
// Pulls together a variety of different algorithms 
// for pathfinding on grid graphs.
//
// @author: dharabor
// @created: 2016-11-23
//

#include "cfg.h"
#include "constants.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jps2_expansion_policy_prune2.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"
#include "timer.h"
#include "nodemap.h"
#include "zero_heuristic.h"

#include "getopt.h"
#include "global.h"

#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <memory>

namespace G = global;
// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;
long long tot = 0;

void
help()
{
    std::cerr 
        << "==> manual <==\n"
        << "This program solves/generates grid-based pathfinding problems using the\n"
        << "map/scenario format from the 2014 Grid-based Path Planning Competition\n\n";

	std::cerr 
    << "The following are valid parameters for SOLVING instances:\n"
	<< "\t--alg [alg] (required)\n"
    << "\t--scen [scen file] (required) \n"
    << "\t--map [map file] (optional; specify this to override map values in scen file) \n"
	<< "\t--checkopt (optional; compare solution costs against values in the scen file)\n"
	<< "\t--verbose (optional; prints debugging info when compiled with debug symbols)\n"
    << "Invoking the program this way solves all instances in [scen file] with algorithm [alg]\n"
    << "Currently recognised values for [alg]:\n"
    << "\tcbs_ll, cbs_ll_w, dijkstra, astar, astar_wgm, astar4c, sipp\n"
    << "\tsssp, jps, jps2, jps+, jps2+, jps, jps4c\n"
    << "\tdfs, gdfs\n\n"
    << ""
    << "The following are valid parameters for GENERATING instances:\n"
    << "\t --gen [map file (required)]\n"
    << "Invoking the program this way generates at random 1000 valid problems for \n"
    << "gridmap [map file]\n";
}

bool
check_optimality(warthog::solution& sol, warthog::experiment* exp)
{
	uint32_t precision = 2;
	double epsilon = (1.0 / (int)pow(10, precision)) / 2;
	double delta = fabs(sol.sum_of_edge_costs_ - exp->distance());

	if( fabs(delta - epsilon) > epsilon)
	{
		std::stringstream strpathlen;
		strpathlen << std::fixed << std::setprecision(exp->precision());
		strpathlen << sol.sum_of_edge_costs_;

		std::stringstream stroptlen;
		stroptlen << std::fixed << std::setprecision(exp->precision());
		stroptlen << exp->distance();

		std::cerr << std::setprecision(exp->precision());
		std::cerr << "optimality check failed!" << std::endl;
		std::cerr << std::endl;
		std::cerr << "optimal path length: "<<stroptlen.str()
			<<" computed length: ";
		std::cerr << strpathlen.str()<<std::endl;
		std::cerr << "precision: " << precision << " epsilon: "<<epsilon<<std::endl;
		std::cerr<< "delta: "<< delta << std::endl;
		exit(1);
	}
    return true;
}

void
run_experiments(warthog::search* algo, std::string alg_name,
        warthog::scenario_manager& scenmgr, bool verbose, bool checkopt,
        std::ostream& out)
{
	/* std::cout  */
  /*       << "id\talg\texpanded\tinserted\tupdated\ttouched\tsurplus" */
  /*       << "\tnanos\tpcost\tplen\tmap\n"; */
	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tscnt\tsfile\n";
  tot = 0;
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		uint32_t startid = exp->starty() * exp->mapwidth() + exp->startx();
		uint32_t goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
        warthog::problem_instance pi(startid, goalid, verbose);
        warthog::solution sol;
        G::statis::clear();
        algo->get_path(pi, sol);

		out
            << i<<"\t" 
            << alg_name << "\t" 
            << sol.nodes_expanded_ << "\t" 
            << sol.nodes_inserted_ << "\t"
            << sol.nodes_touched_ << "\t"
            << sol.time_elapsed_nano_ << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << G::statis::scan_cnt << "\t"
            << scenmgr.last_file_loaded() 
            << std::endl;

    tot += G::statis::scan_cnt;
        if(checkopt) { check_optimality(sol, exp); }
	}
}


void
run_jps2(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jps2_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps2_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    tot = 0;
    G::nodepool = expander.get_nodepool();
    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() 
            << ", tot scan: " << tot << "\n";
}
void
run_jps2_prune2(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
  warthog::gridmap map(mapname.c_str());
	warthog::jps2_expansion_policy_prune2 expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
  warthog::pqueue_min open;

	warthog::flexible_astar<
	  warthog::octile_heuristic,
	  warthog::jps2_expansion_policy_prune2,
    warthog::pqueue_min> astar(&heuristic, &expander, &open);

  tot = 0;
  G::query::map = &map;
  G::query::open = &open;
  G::nodepool = expander.get_nodepool();
  run_experiments(&astar, alg_name, scenmgr, verbose, checkopt, std::cout);
  std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << ", tot scan: " << tot << "\n";
}

void
run_jps(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jps_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_astar(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_dijkstra(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic;
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::zero_heuristic,
	   	warthog::gridmap_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"alg",  required_argument, 0, 1},
		{"scen",  required_argument, 0, 0},
		{"map",  required_argument, 0, 1},
		{"gen", required_argument, 0, 3},
		{"help", no_argument, &print_help, 1},
		{"checkopt",  no_argument, &checkopt, 1},
		{"verbose",  no_argument, &verbose, 1},
		{0,  0, 0, 0}
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "a:b:c:def", valid_args);

    if(argc == 1 || print_help)
    {
		help();
        exit(0);
    }

    std::string sfile = cfg.get_param_value("scen");
    std::string alg = cfg.get_param_value("alg");
    std::string gen = cfg.get_param_value("gen");
    std::string mapname = cfg.get_param_value("map");

	if(gen != "")
	{
		warthog::scenario_manager sm;
		warthog::gridmap gm(gen.c_str());
		sm.generate_experiments(&gm, 1000) ;
		sm.write_scenario(std::cout);
        exit(0);
	}

    // running experiments
	if(alg == "" || sfile == "")
	{
        help();
		exit(0);
	}

    // load up the instances
	warthog::scenario_manager scenmgr;
	scenmgr.load_scenario(sfile.c_str());

    if(scenmgr.num_experiments() == 0)
    {
        std::cerr << "err; scenario file does not contain any instances\n";
        exit(0);
    }

    // the map filename can be given or (default) taken from the scenario file
    if(mapname == "")
    { mapname = scenmgr.get_experiment(0)->map().c_str(); }
    else if(alg == "jps2")
    {
        run_jps2(scenmgr, mapname, alg);
    }
    else if (alg == "jps2-prune2")
    {
      run_jps2_prune2(scenmgr, mapname, alg);
    }
    else if(alg == "jps")
    {
        run_jps(scenmgr, mapname, alg);
    }
    else if(alg == "dijkstra")
    {
        run_dijkstra(scenmgr, mapname, alg); 
    }

    else if(alg == "astar")
    {
        run_astar(scenmgr, mapname, alg); 
    }
    else
    {
        std::cerr << "err; invalid search algorithm: " << alg << "\n";
    }
}


