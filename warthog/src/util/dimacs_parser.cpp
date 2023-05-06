#include "constants.h"
#include "dimacs_parser.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>

warthog::dimacs_parser::dimacs_parser()
{
    init();
}

warthog::dimacs_parser::dimacs_parser(const char* gr_file)
{
    init();
    load_graph(gr_file);
}

warthog::dimacs_parser::dimacs_parser(const char* co_file, 
        const char* gr_file)
{
    init();
    load_graph(co_file);
    load_graph(gr_file);
}

warthog::dimacs_parser::~dimacs_parser()
{
    delete nodes_;
    delete edges_;
    delete experiments_;
}

void
warthog::dimacs_parser::init()
{
    nodes_ = new std::vector<warthog::dimacs_parser::node>();
    edges_ = new std::vector<warthog::dimacs_parser::edge>();
    experiments_ = new std::vector<warthog::dimacs_parser::experiment>();
}

bool
warthog::dimacs_parser::load_graph(const char* filename)
{
    std::fstream* fdimacs = new std::fstream(filename, std::fstream::in);
	if(!fdimacs->is_open())
	{
		std::cerr << "err; dimacs_parser::dimacs_parser "
			"cannot open file: "<<filename << std::endl;
		return false;
	}

	bool retval = true;
	char* buf = new char[1024];
	const char* delim = " \t";
	uint32_t line = 1;
	while(fdimacs->good())
	{
		fdimacs->getline(buf, 1024);
		if(buf[0] == 'c')
            continue;

		if(buf[0] == 'p')
		{
			char* token = strtok(buf, delim); // p char
			token = strtok(NULL, delim); // file type token
			if(strcmp(token, "sp") == 0)
			{
                char* end;
                token = strtok(NULL, delim); 
                uint32_t tmp_num_nodes = (uint32_t)strtol(token, &end, 10);
                
                token = strtok(NULL, delim); 
                uint32_t tmp_num_edges = (uint32_t)strtol(token, &end, 10);
                if(tmp_num_edges == 0L || tmp_num_nodes == 0L)
                {
                    std::cerr 
                        << "error; invalid graph description on line " 
                        << line << " of file " << filename << "\n";
                    delete fdimacs;
                    delete [] buf;
                    return 0;
                }
                std::cerr 
                    << "loading " << tmp_num_edges << " arcs "
                    << "from "<< filename << " ... ";
                edges_->reserve(tmp_num_edges);
				retval = load_gr_file(*fdimacs);
                gr_file_ = filename;
                assert(tmp_num_edges == get_num_edges());
                std::cerr << "done\n";
                break;
			}
			else if(strcmp(token, "aux") == 0)
			{
                token = strtok(NULL, delim);  
                if(strcmp(token, "sp") != 0)
                {
                    retval = false;
                }
                else
                { 
                    token = strtok(NULL, delim);  
                    if(strcmp(token, "co") != 0)
                    {
                        std::cerr 
                            << "error; invalid graph description on line " 
                            << line <<" of file " << filename << "\n";
                        delete fdimacs;
                        delete [] buf;
                        return 0;
                    }
                    else
                    { 
                        token = strtok(NULL, delim);  
                        char* end;
                        uint32_t tmp_num_nodes = (uint32_t)strtol(token, &end, 10);
                        if(tmp_num_nodes == 0L) 
                        { 
                            std::cerr 
                                << "error; invalid graph description on line " 
                                << line <<" of file " << filename << "\n";
                            delete fdimacs;
                            delete [] buf;
                            return 0;
                        }
                        std::cerr 
                            << "loading " << tmp_num_nodes << " nodes "
                            << "from " << filename << " ... ";
                        nodes_->reserve(tmp_num_nodes);
                        retval = load_co_file(*fdimacs);
                        gr_file_ = filename;
                        assert(tmp_num_nodes == get_num_nodes());
                        std::cerr << "done\n";
                        break;
                    }
                }
			}
			else
			{
				std::cerr << "error; unrecognised problem line in dimacs file\n";
                delete fdimacs;
                delete [] buf;
                return 0;
			}
		}
		line++;
	}

    delete fdimacs;
	delete [] buf;
    return retval;
}

bool
warthog::dimacs_parser::load_co_file(std::istream& fdimacs)
{
    nodes_->clear();
    uint32_t line = 1;
	char* buf = new char[1024];
	const char* delim = " \t";
	bool all_good = true;
	while(fdimacs.good() && all_good)
	{
		char next_char = (char)fdimacs.peek();
		switch(next_char)
		{
			case 'v':
			{
				fdimacs.getline(buf, 1024);
				char* descriptor = strtok(buf, delim);
				char* id = strtok(NULL, delim);
				char* x = strtok(NULL, delim);
				char* y = strtok(NULL, delim);
				if(!(descriptor && id && x && y))
				{
					std::cerr << "warning; badly formatted node descriptor on line "
                        <<line << std::endl;
                    all_good = false;
					break;
				}
                warthog::dimacs_parser::node n;
                n.id_ = (uint32_t)atoi(id);
                n.x_ = atoi(x);
                n.y_ = atoi(y);
                nodes_->push_back(n);
				break;
			}
			case 'p': // stop if we hit another problem line
				all_good = false;
				break;
			default: // ignore non-node, non-problem lines
				fdimacs.getline(buf, 1024);
				break;
		}
//        if((line % 10000) == 0)
//        {
//                std::cerr << "\r co file; K lines read: " << (line / 1000) << " ";
//        }
		line++;
	}

	delete [] buf;
	return all_good;
}

bool
warthog::dimacs_parser::load_gr_file(std::istream& fdimacs)
{
    uint32_t line = 1;
	char* buf = new char[1024];
	const char* delim = " \t";
	bool all_good = true;
	while(fdimacs.good() && all_good)
	{
		char next_char = (char)fdimacs.peek();
		switch(next_char)
		{
			case 'a':
			{
				fdimacs.getline(buf, 1024);
				char* descriptor = strtok(buf, delim);
				char* from = strtok(NULL, delim);
				char* to = strtok(NULL, delim);
				char* cost = strtok(NULL, delim);
				if(!(descriptor && from && to && cost))
				{
					std::cerr << "warning; badly formatted arc descriptor on line "
                        <<line << std::endl;
                    all_good = false;
					break;
				}
                warthog::dimacs_parser::edge e;
                e.tail_id_ = (uint32_t)atoi(from);
                e.head_id_ = (uint32_t)atoi(to);
                e.weight_ = (uint32_t)atoi(cost);
                edges_->push_back(e);
				break;
			}
			case 'p': // another problem line. stop here
				all_good = false;
				break;
			default: // ignore non-arc, non-problem lines
				fdimacs.getline(buf, 1024);
				break;
		}
//        if((line % 10000) == 0)
//        {
//                std::cerr << "\r gr file; K lines read: " << (line / 1000) << " ";
//        }
		line++;
	}

	delete [] buf;
	return all_good;
}

void
warthog::dimacs_parser::print(std::ostream& oss)
{
    uint32_t nnodes = (uint32_t)nodes_->size();
    if(nnodes > 0)
    {
        oss << "p aux sp co " << nodes_->size() << std::endl;
        for(uint32_t i = 0; i < nnodes; i++)
        {
            warthog::dimacs_parser::node n = nodes_->at(i);
            oss << "v " << i+1 << " " << n.x_ << " " << n.y_ << std::endl;
        }
    }

    uint32_t nedges = (uint32_t)edges_->size();
    if(nedges > 0)
    {
        oss << "p sp " << nnodes << " " << nedges << std::endl;
        for(uint32_t i = 0; i < nedges; i++)
        {
            warthog::dimacs_parser::edge e = edges_->at(i);
            oss << "a " << e.tail_id_ << " " << e.head_id_ << " " << e.weight_ << std::endl;
        }
    }

}

bool
warthog::dimacs_parser::load_instance(const char* dimacs_file)
{
    problemfile_ = std::string(dimacs_file);
	std::ifstream infile;
	infile.open(dimacs_file,std::ios::in);


    bool p2p = true;
    char buf[1024];

    // dimacs stuff is 1-indexed but warthog is zero indexed
    // we adjust ids if necessary 
    uint32_t id_offset = 1; 
    while(infile.good())
    {
        infile.getline(buf, 1024);
        // skip comment lines
        if(buf[0] == 'c')
        {
            continue;
        }
        // "p2p-xy is a warthog extended problem specification format
        // where start and target ids are zero indexed. In these cases
        // we don't need to adjust the start and target ids
        else if(strstr(buf, "p aux sp p2p-zero") != 0)
        {
            p2p = true;
            id_offset = 0;
            break;
        }
        else if(strstr(buf, "p aux sp ss-zero") != 0)
        {
            id_offset = 0;
            p2p = false;
            break;
        }
        if(strstr(buf, "p aux sp p2p") != 0)
        {
            p2p = true;
            break;
        }
        else if(strstr(buf, "p aux sp ss") != 0)
        {
            p2p = false;
            break;
        }
    }
        
    infile.getline(buf, 1024);
    while(infile.good())
    {
        if(buf[0] == 'c')
        {
            infile.getline(buf, 1024);
            continue;
        }

        char* tok = strtok(buf, " \t\n");
        if(strcmp(tok, "q") == 0 || strcmp(tok, "s") == 0)
        {
            warthog::dimacs_parser::experiment exp;

            tok = strtok(0, " \t\n");
            if(tok == 0)
            {
                std::cerr << "skipping invalid query in problem file:  " 
                    << buf << "\n";
                continue;

            }
            exp.source = (uint32_t)atoi(tok);
            exp.source -= id_offset;
            exp.p2p = p2p;

            if(p2p)
            {
                tok = strtok(0, " \t\n");
                if(tok == 0)
                {
                    std::cerr << "invalid query in problem file:  " << buf << "\n";
                }
                exp.target = (uint32_t)atoi(tok);
                exp.target -= id_offset;
            }
            else
            {
                exp.target = warthog::INF32;
            }

            // add the experiment
            experiments_->push_back(exp);
        } 
        else
        {
            std::cerr << "skipping invalid query in problem file: " 
                << buf << std::endl;
        }
        infile.getline(buf, 1024);
    }
    //std::cerr << "loaded "<<experiments_->size() << " queries\n";
    return true;
}

void
warthog::dimacs_parser::print_undirected_unweighted_metis(std::ostream& out,
                double core_pct_value, std::vector<uint32_t>* lex_order)
{
    std::unordered_map<uint32_t, std::set<uint32_t>> adj;
    std::set<uint32_t> nodes;

    uint32_t num_undirected_edges = 0;
    uint32_t id_offset = 1; // order ids begin from 0; dimacs ids from 1

    // enumerate all the nodes
    for(uint32_t i = 0; i < edges_->size(); i++)
    {
        warthog::dimacs_parser::edge e = edges_->at(i);
        uint32_t id1 = e.head_id_;
        uint32_t id2 = e.tail_id_;
        bool include_id1 = true, include_id2 = true;

        // filtering stuff for when the DIMACS graph is a CH and we want to 
        // convert just a subset of the graph (== a core graph)
        if(lex_order)
        {
            // core is the top-k% of nodes; 
            uint32_t min_core_level = (uint32_t)((double)lex_order->size() * (1-core_pct_value));

            // metis ids need to be contiguous and start from 1
            if(lex_order->at(id1 - id_offset) >= min_core_level)
            { 
                id1 = id1 - min_core_level;
                assert(id1 < lex_order->size());
            }
            else { include_id1 = false; }

            if(lex_order->at(id2 - id_offset) >= min_core_level)
            { 
                id2 = id2 - min_core_level;
                assert(id2 < lex_order->size());
            }
            else { include_id2 = false; }
        }

        if(include_id1) { nodes.insert(id1); }
        if(include_id2) { nodes.insert(id2); }

        if(adj.find(id1) == adj.end())
        {
            std::set<uint32_t> neis;
            std::pair<uint32_t, std::set<uint32_t>> elt(id1, neis);
            adj.insert(elt);
        }
        if(adj.find(id2) == adj.end())
        {
            std::set<uint32_t> neis;
            std::pair<uint32_t, std::set<uint32_t>> elt(id2, neis);
            adj.insert(elt);
        }
        
        if(include_id1 && include_id2)
        {
            std::set<uint32_t>& neis1 = adj.find(id1)->second;
            std::set<uint32_t>& neis2 = adj.find(id2)->second;
            if(neis1.find(id2) == neis1.end() && neis2.find(id1) == neis2.end())
            {
                num_undirected_edges++;
            }

            auto iter1 = adj.find(id1);
            auto iter2 = adj.find(id2);
            assert(iter1 != adj.end() && iter2 != adj.end());
            iter1->second.insert(id2);
            iter2->second.insert(id1);
        }
    }
    std::cerr << "conversion done; " << nodes.size() << " nodes and " << num_undirected_edges << " edges; printing\n";

    // ids need to be a contiguous series, from 1 to nodes.size()
    // here we convert the identifiers (since the input file may 
    // not be dimacs compliant in this respect)
    std::unordered_map<uint32_t, uint32_t> id_map;
    uint32_t current_id = 1;
    for(std::set<uint32_t>::iterator it = nodes.begin(); 
            it != nodes.end(); it++)
    {
        id_map.insert(std::pair<uint32_t, uint32_t>(*it, current_id));
        current_id++;
    }

    out << nodes.size() << " " << num_undirected_edges << std::endl;
    for(auto it = nodes.begin(); it != nodes.end(); it++)
    {
        std::set<uint32_t>& neis = adj.find(*it)->second;
        for(auto nit = neis.begin();  nit != neis.end(); nit++)
        {
            out << (*id_map.find((*nit))).second << " "; 
        }
        out << std::endl;
    }
}
