#ifndef WARTHOG_DIMACS_PARSER_H
#define WARTHOG_DIMACS_PARSER_H

// dimacs_parser.h
//
// A parser for reading road networks of the type
// used at the 9th DIMACS competition.
//
// @author: dharabor
// @created: 2015-01-08
//

#include "problem_instance.h"

#include <stdint.h>
#include <vector>
#include <fstream>


namespace warthog
{

class dimacs_parser
{
    public:
        struct node
        {
            uint32_t id_;
            int32_t x_;
            int32_t y_;
        };

        struct edge
        {
            uint32_t tail_id_;
            uint32_t head_id_;
            uint32_t weight_;

            bool 
            operator<(warthog::dimacs_parser::edge& other)
            {
                return head_id_ < other.head_id_;
            }
        };

        struct experiment
        {
            uint32_t source;
            uint32_t target;
            bool p2p; // true indicates point-to-point instance; else SSSP

            warthog::problem_instance
            get_instance() 
            { return warthog::problem_instance(source, target); }
        };


        typedef  std::vector<warthog::dimacs_parser::node>::iterator node_iterator;
        typedef  std::vector<warthog::dimacs_parser::edge>::iterator edge_iterator;
        typedef  std::vector<warthog::dimacs_parser::experiment>::iterator experiment_iterator;

        dimacs_parser();
        dimacs_parser(const char* gr_file);
        dimacs_parser(const char* co_file, const char* gr_file);
        ~dimacs_parser();
        
        // load up a DIMACS file (gr or co)
        // NB: upon invocation, this operation will discard all current nodes
        // (or edges, depending on the type of file passed for loading) and 
        // THEN attempt to load new data.
        bool 
        load_graph(const char* dimacs_file);

        bool
        load_instance(const char* dimacs_file);

        inline uint32_t
        get_num_nodes() 
        {
           return (uint32_t)nodes_->size();
        }

        inline uint32_t
        get_num_edges()
        {
            return (uint32_t)edges_->size();
        }

        inline warthog::dimacs_parser::node_iterator
        nodes_begin()
        {
            return nodes_->begin();
        }

        inline warthog::dimacs_parser::node_iterator
        nodes_end()
        {
            return nodes_->end();
        }

        inline warthog::dimacs_parser::edge_iterator
        edges_begin()
        {
            return edges_->begin();
        }
        
        inline warthog::dimacs_parser::edge_iterator
        edges_end()
        {
            return edges_->end();
        }

        inline warthog::dimacs_parser::experiment_iterator
        experiments_begin()
        {
            return experiments_->begin();
        }

        inline uint32_t 
        num_experiments() { return (uint32_t)experiments_->size(); }



        inline warthog::dimacs_parser::experiment_iterator
        experiments_end()
        {
            return experiments_->end();
        }

        void
        print(std::ostream&);

        // convert to the graph format used by the METIS 
        // graph partitioning library
        void 
        print_undirected_unweighted_metis(std::ostream&, 
                double core_pct_value=1, std::vector<uint32_t>* lex_order=0);

        inline std::string 
        get_problemfile() { return problemfile_; }

        inline std::string
        get_co_filename() { return co_file_; } 

        inline std::string
        get_gr_filename() { return gr_file_; }

    private:
        void init();
        bool load_co_file(std::istream& fdimacs);
        bool load_gr_file(std::istream& fdimacs);

       std::vector<warthog::dimacs_parser::node>* nodes_;
       std::vector<warthog::dimacs_parser::edge>* edges_;
       std::vector<warthog::dimacs_parser::experiment>* experiments_;
       std::string problemfile_;
       std::string co_file_;
       std::string gr_file_;

};

}

#endif

