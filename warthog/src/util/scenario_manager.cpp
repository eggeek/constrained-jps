#include "dummy_listener.h"
#include "flexible_astar.h"
#include "scenario_manager.h"
#include "problem_instance.h"

#include <cstdlib>
#include <cstring>

static uint32_t head_offset = 0;
static uint32_t tail_offset = 0;
static const int MAXTRIES=10000000;

warthog::scenario_manager::scenario_manager() 
{
}

warthog::scenario_manager::~scenario_manager()
{
	for(unsigned int i=0; i < experiments_.size(); i++)
	{
		delete experiments_[i];
	}
	experiments_.clear();
}

void 	
warthog::scenario_manager::generate_experiments(
		warthog::gridmap* map, int num) 
{
	assert(map); // need a test here; throw exception if absMap is null
	
	head_offset = tail_offset = 0;
	int tries=0;
	int generated=0;
	while(generated < num)
	{	
		if(tries >= MAXTRIES)
		{
			std::cerr << "err; scenario_manager::generate_experiments"
				<< " tried "<<tries << " times but could not generate "
				"a valid experiment. giving up.\n";
			exit(1);
		}
		
		experiment* exp = generate_single_experiment(map); 
		if(exp != NULL) 
		{
			this->add_experiment(exp);
			generated++;
			if((generated % 10) == 0)
			{
				head_offset += 10;
				tail_offset += 20;
				std::cerr << "\rgenerated: "<< generated << "/" << num;
				std::cerr << std::flush;
			}
		}
		tries++;
		if((tries % 5) == 0)
		{
			head_offset += 5;
			tail_offset += 10;
		}
	}
	std::cerr << " experiments." << std::endl;
	sort();
}

warthog::experiment* 
warthog::scenario_manager::generate_single_experiment(warthog::gridmap* map)
{
	warthog::gridmap_expansion_policy expander(map);
	warthog::octile_heuristic heuristic(map->width(), map->height());
    warthog::pqueue_min open;
    warthog::dummy_listener listener;
	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::gridmap_expansion_policy, 
        warthog::pqueue_min, warthog::dummy_listener>
            astar(&heuristic, &expander, &open, &listener);

	warthog::experiment* newexp;
	uint32_t raw_mapsize = map->header_height() * map->header_width();

	uint32_t window_size = (uint32_t)(raw_mapsize * 0.01);
	if(head_offset + window_size >= raw_mapsize)
		head_offset = 0;
	if(tail_offset + window_size >= raw_mapsize)
		tail_offset = 0;

	uint32_t id1 = 
        map->to_padded_id(
            (((uint32_t)rand() % window_size) + head_offset) % raw_mapsize);
	uint32_t id2 = 
        map->to_padded_id(
            (raw_mapsize - (((uint32_t)rand() % window_size) + tail_offset)) % raw_mapsize);
    warthog::problem_instance pi(id1, id2);
    warthog::solution sol;
	astar.get_path(pi, sol);

	if(sol.sum_of_edge_costs_ == warthog::INF32)
	{
	//	std::cout << " no path;" <<std::endl;
		return 0;
	}
	//std::cout << " found path;" << std::endl;
		
	uint32_t x1, x2, y1, y2;
	map->to_unpadded_xy(id1,x1,y1);
	map->to_unpadded_xy(id2, x2, y2);
	newexp = new experiment(x1, y1, x2, y2, 
			map->header_width(), map->header_height(), 
			sol.sum_of_edge_costs_, std::string(map->filename()));
	return newexp;
}

void 
warthog::scenario_manager::load_scenario(const char* filelocation)
{
	std::ifstream infile;
	infile.open(filelocation,std::ios::in);

	if(!infile.good())
	{
		std::cerr << "err; scenario_manager::load_scenario "
		<< "Invalid scenario file: "<<filelocation << std::endl;
		infile.close();
		exit(1);
	}

	sfile_ = filelocation;

    char buf[1024];
    infile.getline(buf, 1024);
    if(strstr(buf, "version 1"))
	{
        // GPPC format scenarios
		load_gppc_scenario(infile);
	}
	else
	{
		std::cerr << "err; scenario_manager::load_scenario "
			<< " scenario file not in GPPC format\n";
		infile.close();
		exit(1);
	}
	infile.close();
}

// V1.0 is the version officially supported by HOG
void 
warthog::scenario_manager::load_gppc_scenario(std::ifstream& infile)
{
	uint32_t sizeX = 0, sizeY = 0; 
	uint32_t bucket;
	std::string map;  
	uint32_t xs, ys, xg, yg;
	std::string dist;

	while(infile>>bucket>>map>>sizeX>>sizeY>>xs>>ys>>xg>>yg>>dist)
	{
		double dbl_dist = strtod(dist.c_str(),0);
		experiments_.push_back(
				new experiment(xs,ys,xg,yg,sizeX,sizeY,dbl_dist,map));

		int32_t precision = 0;
		if(dist.find(".") != std::string::npos)
		{
			precision = ((int32_t)dist.size() - (int32_t)(dist.find(".")+1));
		}
		experiments_.back()->set_precision(precision);
	}

    // sanity check
    if(experiments_.size() == 0) { return; }
}

void 
warthog::scenario_manager::write_scenario(std::ostream& scenariofile)
{

	std::cerr << "dumping scenario file..\n";
	if(experiments_.size() == 0) // nothing to write
		return;

	//std::ofstream scenariofile;
	scenariofile.precision(16);
	//scenariofile.open(filelocation, std::ios::out);
	scenariofile << "version 1" << std::endl;

	for(unsigned int i=0; i<experiments_.size(); i++)
	{	
		experiment*	cur = experiments_.at(i);
		cur->print(scenariofile);
		scenariofile << std::endl;
	}
	//scenariofile.close();		
}

void 
warthog::scenario_manager::sort()
{
	for(unsigned int i=0; i < experiments_.size(); i++)
	{
		for(unsigned int j = i; j < experiments_.size(); j++)
		{
			if(experiments_.at(j)->distance() <
				   	experiments_.at(i)->distance())
			{
				experiment* tmp = experiments_.at(i);
				experiments_.at(i) = experiments_.at(j);
				experiments_.at(j) = tmp;
			}
		}
	}
}
