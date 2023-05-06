#ifndef WARTHOG_XY_GRAPH_H
#define WARTHOG_XY_GRAPH_H

// xy_graph.h
//
// A simple general purpose data structure for directed weighted xy graphs
// (i.e. spatial networks embedded in two dimensions).
// Supported input types are:
//  - warthog::gridmap objects
//  - road network graphs in the format of the 9th DIMACS competition
//
// This implemention stores all nodes and edges in two separate flat arrays
// and uses one to index the other. The graph can contain a maximum of
// 2^32 nodes and edges.
//
// @author: dharabor
// @created: 2016-01-07
//

#include "cast.h"
#include "constants.h"
#include "dimacs_parser.h"
#include "euclidean_heuristic.h"
#include "forward.h"
#include "graph.h"
#include "gridmap_expansion_policy.h"
#include "util/timer.h"

#include <ostream>
#include <unordered_map>
#include <vector>

namespace warthog
{

namespace graph
{

// Need to be declared before 'xy_graph_base'
void
parse_xy(
    std::istream& in, uint32_t& num_nodes, uint32_t& num_edges,
    std::vector<std::pair<uint32_t, warthog::graph::edge>>& edges,
    std::vector<std::pair<int32_t, int32_t>>& xy,
    std::vector<warthog::graph::ECAP_T>& in_degree,
    std::vector<warthog::graph::ECAP_T>& out_degree);

template<class T_NODE, class T_EDGE>
class xy_graph_base
{
    public:
        // create an empty graph
        xy_graph_base(
            uint32_t num_nodes = 0, std::string filename = "",
            bool incoming = false)
            : verbose_(false), filename_(filename), store_incoming_(incoming),
            graph_id_(graph_counter_++)
        {
            grow(num_nodes);
        }

        ~xy_graph_base() { }

        // TODO If you need the copy constructor, you will need to drop the
        // @delete@ as we need to update 'graph_counter_' to be consistent.
        xy_graph_base(const xy_graph_base&) = delete;

        warthog::graph::xy_graph_base<T_NODE, T_EDGE>&
        operator=(const warthog::graph::xy_graph_base<T_NODE, T_EDGE>&& other)
        {
            // my memory, not want
            clear();

            // your memory, can has?
            verbose_ = other.verbose_;
            filename_ = other.filename_;
            nodes_ = std::move(other.nodes_);
            xy_ = std::move(other.xy_);
            // Technically the same but don't know what you will do with it.
            graph_id_ = graph_counter_++;

            return *this;
        }

        bool
        operator==(const warthog::graph::xy_graph_base<T_NODE, T_EDGE>& other)
        {
            if(xy_.size() == other.xy_.size())
            {
                for(uint32_t i = 0; i < xy_.size(); i++)
                {
                    if(xy_[i] != other.xy_[i]) { return false; }
                }
            }
            else
            {
                return false;
            }

            if(get_num_nodes() == other.get_num_nodes())
            {
                for(uint32_t i = 0; i < get_num_nodes(); i++)
                {
                    if(!(nodes_[i] == other.nodes_[i])) { return false; }
                }
                return true;
            }
            return false;
        }

        // set the number of nodes and edges in the graph to zero.
        // any currently assigned nodes and edges have their destructors
        // called
        void
        clear()
        {
            nodes_.clear();
            xy_.clear();
        }

        // grow the graph so that the number of vertices is equal to
        // @param num_nodes. if @param num_nodes is less than the current
        // number of nodes, this function does nothing.
        void
        grow(size_t num_nodes)
        {
            if(num_nodes <= nodes_.size()) { return; }
            nodes_.resize(num_nodes);
            xy_.resize(num_nodes*2, INT32_MAX);
        }

        // allocate capacity for at least @param num_nodes
        // when @param num_nodes <= the number of nodes in the graph, this
        // function does nothing
        void
        capacity(size_t num_nodes)
        {
            nodes_.reserve(num_nodes);
            xy_.reserve(num_nodes*2);
        }

        inline uint32_t
        get_id() const
        {
            return graph_id_;
        }

        inline uint32_t
        get_num_nodes() const
        {
            return (uint32_t)nodes_.size();
        }

        inline uint32_t
        get_num_edges_out() const
        {
            uint32_t num_edges = 0;
            for(auto& node : nodes_)
            {
                num_edges += node.out_degree();
            }
            return num_edges;
        }

        inline uint32_t
        get_num_edges_in() const
        {
            uint32_t num_edges = 0;
            for(auto& node : nodes_)
            {
                num_edges += node.in_degree();
            }
            return num_edges;
        }

        // Fetch the xy coordinates of a node
        //
        // @param id: an internal graph id
        // @return x: the x coordinate of node @param id
        // @return y: the y coordinate of node @param id
        inline void
        get_xy(uint32_t id, int32_t& x, int32_t& y) const
        {
            assert(id < (xy_.size()>>1) );
            x = xy_[id*2];
            y = xy_[id*2+1];
        }

        // Set the xy coordinates of a node
        //
        // @param id: an internal graph id
        // @param x: the x coordinate of node @param id
        // @param y: the y coordinate of node @param id
        inline void
        set_xy(uint32_t id, int32_t x, int32_t y)
        {
            uint32_t nodes_sz = get_num_nodes();
            if(id >= nodes_sz) { return; }
            if(xy_.size() != nodes_sz) { xy_.resize(nodes_sz*2); }
            xy_[id*2] = x;
            xy_[id*2+1] = y;
        }

        // Fetch a node
        //
        // @param id: an internal graph id
        // @return: the node object associated with @param id
        inline T_NODE*
        get_node(uint32_t id)
        {
            //if(id >= ID_OFFSET && id < nodes_sz_)
            if(id < get_num_nodes())
            {
                return &nodes_[id];
            }
            return 0;
        }

        // Add a new node into the graph. If a node already exists in the
        // graph with the same external id as @param ext_id then then nothing
        // is added.
        // NB: a new node is always added if @param ext_id is equal to the
        // value warthog::INF
        //
        // @param x: the x-coordinate of the new node
        // @param y: the y-coordinate of the new node
        // @param ext_id: an (optional) external id for this node
        // @return: the internal graph id of the new node or the id of the
        // existing node whose graph id is equal to @param ext_id
        uint32_t
        add_node(int32_t x, int32_t y, uint32_t ext_id)
        {
            // check if a node with the same external id already exists; if so
            // return the id of the existing node. otherwise, add a new node
            //uint32_t graph_id = to_graph_id(ext_id);
            //if(graph_id != warthog::INF32) { return graph_id; }

            uint32_t graph_id = (uint32_t)get_num_nodes();
            nodes_.push_back(T_NODE());
            xy_.push_back(x);
            xy_.push_back(y);
            return graph_id;
        }

        uint32_t
        add_node(int32_t x, int32_t y)
        {
            return add_node(x, y, warthog::INF32);
        }

        uint32_t
        add_node()
        {
            return add_node(warthog::INF32, warthog::INF32, warthog::INF32);
        }

        // print extra stuff to std::err
        inline void
        set_verbose(bool verbose) { verbose_ = verbose; }

        inline bool
        get_verbose() const { return verbose_; }

        // @return the name of the file from which the
        // current graph object was constructed
        inline const char*
        get_filename() const { return filename_.c_str(); }

        inline void
        set_filename(const char* filename)
        {
            filename_ = filename;
        }

        inline size_t
        mem()
        {
            size_t mem = 0;
            for(uint32_t i = 0; i < get_num_nodes(); i++)
            {
                mem += nodes_[i].mem();
            }
            mem += sizeof(int32_t) * xy_.size() * 2;
            mem += sizeof(char)*filename_.length() +
                sizeof(*this);
            return mem;
        }

        // convert an external node id (e.g. as it appears in an input file)
        // to the equivalent internal id used by the current graph
        //
        // @param ex_id: the external id of the node
        // @return: the corresponding internal id for @param ex_id
        // if ex_id did not appear in the input file (or if the graph
        // was not created from an input file) the function returns
        // the value warthog::INF32
        inline uint32_t
        to_graph_id(uint32_t ext_id)
        {
            return ext_id;
        }

        // convert an internal node id (i.e. as used by the current graph
        // to the equivalent external id (e.g. as appears in an input file)
        //
        // @param ex_id: the external id of the node
        // @return: the corresponding internal id for @param ex_id
        // if ex_id did not appear in the input file (or if the graph
        // was not created from an input file) the function returns
        // the value warthog::INF32
        inline uint32_t
        to_external_id(uint32_t in_id)  const
        {
            return in_id;
        }

        // compute the proportion of bytes allocated to edges with respect
        // to the size of the address space those bytes span.
        // a value of 1 using this metric indicates that the edges perfectly
        // fit into the allocated address space
        inline double
        edge_mem_frag()
        {
            T_EDGE *min_addr, *max_addr;
            min_addr = this->get_node(0)->outgoing_begin();
            max_addr = this->get_node(0)->outgoing_end();
            for(uint32_t i = 0; i < this->get_num_nodes(); i++)
            {
                T_NODE* n = this->get_node(i);
                T_EDGE* out_begin = n->outgoing_begin();
                T_EDGE* out_end = n->outgoing_end();
                T_EDGE* in_begin = n->incoming_begin();
                T_EDGE* in_end = n->incoming_end();

                min_addr = out_begin ?
                            (out_begin < min_addr ? out_begin : min_addr) :
                            min_addr;
                max_addr = out_end ?
                            (out_end > max_addr ? out_end : max_addr) :
                            max_addr;
                min_addr = in_begin ?
                            (in_begin < min_addr ? in_begin : min_addr) :
                            min_addr;
                max_addr = in_end ?
                            (in_end > max_addr ? in_end : max_addr) :
                            max_addr;
            }

            size_t mem_lb = sizeof(T_EDGE) *
                (this->get_num_edges_out() + this->get_num_edges_in());
            size_t mem_actual =
                ((size_t)(max_addr-min_addr)) * sizeof(T_EDGE);
            return (double)mem_actual / (double)mem_lb;
        }


        // check if arc weights are Euclidean and (optionally) fix if not.
        // "fixing" means arc weights must be at least as large as the
        // Euclidean distance between the arc's head and tail vertex.
        bool
        is_euclidean(bool fix_if_not=true)
        {
            warthog::euclidean_heuristic h_euc(this);
            for(uint32_t t_id= 0; t_id < get_num_nodes(); t_id++)
            {
                int32_t tx, ty, hx, hy;
                get_xy(t_id, tx, ty);

                for(T_EDGE* it = nodes_[t_id].outgoing_begin();
                    it != nodes_[t_id].outgoing_end();
                    it++)
                {
                    uint32_t h_id = (*it).node_id_;
                    get_xy(h_id, hx, hy);

                    warthog::cost_t hdist = h_euc.h(tx, ty, hx, hy);
                    if((*it).wt_ < hdist)
                    {
                        if(!fix_if_not) { return false; }
                        (*it).wt_ = static_cast<uint32_t>(ceil(hdist));
                    }
                }
            }
            return true;
        }

        /**
         * Perturb given an already loaded 'xy_graph' and a new file.
         */
        void
        perturb(std::istream& in)
        {
            uint32_t num_nodes = 0;
            uint32_t num_edges = 0;
            std::vector<std::pair<uint32_t, warthog::graph::edge>> edges;
            std::vector<std::pair<int32_t, int32_t>> xy;
            std::vector<warthog::graph::ECAP_T> in_degree;
            std::vector<warthog::graph::ECAP_T> out_degree;

            warthog::graph::parse_xy(
                in, num_nodes, num_edges, edges, xy, in_degree, out_degree);
            assert(num_nodes == nodes_.size());

            perturb(edges);
        }

        /**
         * Perturb given an already loaded 'xy_graph' and a companion graph.
         */
        void
        perturb(xy_graph& g)
        {
            std::vector<std::pair<uint32_t, warthog::graph::edge>> edges;
            assert(nodes_.size() == g.get_num_nodes());

            for(uint32_t i = 0; i < g.get_num_nodes(); i++)
            {
                warthog::graph::node* n = g.get_node(i);
                for(uint32_t edge_idx = 0;
                    edge_idx < n->out_degree();
                    edge_idx++)
                {
                    warthog::graph::edge* e = n->outgoing_begin() + edge_idx;
                    edges.push_back({i, *e});
                }
            }

            perturb(edges);
        }

        /**
         * Edit the weights of the edges to contain the new costs and save the
         * original cost in the labels.
         */
        void
        perturb(std::vector<std::pair<uint32_t, warthog::graph::edge>>& edges)
        {
            uint32_t num_modif = 0;
            for(auto e : edges)
            {
                uint32_t from_id = e.first;
                node* from = get_node(from_id);
                edge_iter eit = from->find_edge(e.second.node_id_);

                if(eit != from->outgoing_end())
                {
                    // Save the original value as the label
                    if(eit->label_ == 0)
                    {
                        eit->label_ = cpd::wt_to_label(eit->wt_);
                    }

                    if(eit->wt_ != e.second.wt_)
                    {
                        num_modif++;
                        eit->wt_ = e.second.wt_;
                    }
                }
                // TODO else add warning (from log.h?)
            }

            // The important bit: update the graph's id when perturbating
            graph_id_ = graph_counter_++;

            std::cerr << "Perturbed " << num_modif << " edges." << std::endl;
        }

        //inline void
        //shrink_to_fit()
        //{
        //    edge* tmp = new edge[get_num_edges()];
        //    uint32_t e_index = 0;
        //    for(uint32_t i = 0; i < this->get_num_nodes(); i++)
        //    {
        //        uint32_t in_deg = nodes_[i].in_degree();
        //        uint32_t out_deg = nodes_[i].out_degree();
        //        nodes_[i].relocate(&tmp[e_index], &tmp[e_index + in_deg]);
        //        e_index += (in_deg + out_deg);
        //    }
        //}

        friend std::istream&
        operator>>(
            std::istream& in, warthog::graph::xy_graph_base<T_NODE, T_EDGE>& g)
        {
            warthog::timer mytimer;
            mytimer.start();

            uint32_t num_nodes = 0, num_edges = 0;
            std::vector<std::pair<uint32_t, warthog::graph::edge>> edges;
            std::vector<std::pair<int32_t, int32_t>> xy;
            std::vector<warthog::graph::ECAP_T> in_degree;
            std::vector<warthog::graph::ECAP_T> out_degree;

            warthog::graph::parse_xy(
                in, num_nodes, num_edges, edges, xy, in_degree, out_degree);
            // allocate memory for nodes
            g.clear();
            g.grow(num_nodes);

            // allocate memory for edges and set xy coordinates
            for(uint32_t i = 0; i < num_nodes; i++)
            {
                g.set_xy(i, xy[i].first, xy[i].second);
                g.get_node(i)->capacity(out_degree[i], in_degree[i]);
            }

            // add edges
            for(std::pair<uint32_t, warthog::graph::edge> e : edges)
            {
                uint32_t from_id = e.first;
                warthog::graph::node* from = g.get_node(from_id);
                from->add_outgoing(e.second);
                // assert(from_id != e.second.node_id_);
                if(g.store_incoming_)
                {
                    uint32_t to_id = e.second.node_id_;
                    warthog::graph::node* to = g.get_node(to_id);
                    e.second.node_id_ = from_id;
                    to->add_incoming(e.second);
                }
            }

            mytimer.stop();
            std::cerr << "graph, loaded.\n";
            std::cerr << "read " << num_nodes << " nodes"
                    << " and read " << num_edges << " outgoing edges"
                    << ". total time "
                    << (double)mytimer.elapsed_time_nano() / 1e9 << " s"
                    << std::endl;

            return in;
        }

        friend std::ostream&
        operator<<(
            std::ostream& out, warthog::graph::xy_graph_base<T_NODE, T_EDGE>& g)
        {
            warthog::timer mytimer;
            mytimer.start();

            // comments
            out << "# warthog xy graph\n"
                << "# this file is formatted as follows: [header data] [node "
                "data] [edge data]\n"
                << "# header format: nodes [number of nodes] edges [number of "
                "edges] \n"
                << "# node data format: v [id] [x] [y]\n"
                << "# edge data format: e [from_node_id] [to_node_id] [cost]\n"
                << "#\n"
                << "# 32bit integer values are used throughout.\n"
                << "# Identifiers are all zero indexed.\n"
                << std::endl;

            // header stuff
            // out << "chd 1.0" << std::endl;
            out << "nodes " << g.get_num_nodes() << " "
                << "edges " << g.get_num_edges_out() << std::endl;

            // node data
            for(uint32_t i = 0; i < g.get_num_nodes(); i++)
            {
                int32_t x, y;
                g.get_xy(i, x, y);
                out << "v " << i << " " << x << " " << y << " " << std::endl;
            }

            for(uint32_t i = 0; i < g.get_num_nodes(); i++)
            {
                warthog::graph::node* n = g.get_node(i);
                for(uint32_t edge_idx = 0; edge_idx < n->out_degree(); edge_idx++)
                {
                    warthog::graph::edge* e = n->outgoing_begin() + edge_idx;
                    out << "e " << i << " " << e->node_id_ << " " << e->wt_
                        << std::endl;
                }
            }

            mytimer.stop();
            std::cerr << "wrote xy_graph; time "
                    << ((double)mytimer.elapsed_time_nano() / 1e9) << " s"
                    << std::endl;

            return out;
        }

      private:
        // the set of nodes that comprise the graph
        std::vector<T_NODE> nodes_;

        // xy coordinates stored as adjacent pairs (x, then y)
        std::vector<int32_t> xy_;

        bool verbose_;
        std::string filename_;
        bool store_incoming_;
        uint32_t graph_id_;
        static uint32_t graph_counter_;
};

template<class T_NODE, class T_EDGE>
uint32_t xy_graph_base<T_NODE, T_EDGE>::graph_counter_ = 0;

typedef xy_graph_base<warthog::graph::node, warthog::graph::edge> xy_graph;

// Create an xy-graph from a gridmap data in the format of
// the Grid-based Path Planning Competition
//
// @param gm: the gridmap
// @param g: the target graph object; if not empty, the graph will be cleared
// @param store_incoming_edges: if true, incoming and outgoing edges are
// both added to the graph; if false, only outgoing edges are added.
void
gridmap_to_xy_graph(
    warthog::gridmap* gm, warthog::graph::xy_graph*,
    bool store_incoming = false);

// create an xy-graph from DIMACS 9th Challenge graph data
// In this format graphs are specified using two files:
// (i) a gr file which defines edge weights and endpoints and;
// (ii) a co file which defines node ids and xy coordinates
//
// @param dimacs: a parser object with preloaded coordinates and edge data
// @param g: the target graph object; if not empty, the graph will be cleared
// @param reverse_arcs: reverses the direction of each edge
// @param store_incoming_edges: store edges with both head and tail node
// @param enforce_euclidean: arc lengths must be >= euclidean distance
void
dimacs_to_xy_graph(
    warthog::dimacs_parser& dimacs, warthog::graph::xy_graph& g,
    bool reverse_arcs = false, bool store_incoming_edges = false,
    bool enforce_euclidean = true);

void
write_dimacs(std::ostream& out, warthog::graph::xy_graph& g);

}
}

#endif
