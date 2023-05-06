#include "heuristics/zero_heuristic.h"
#include "label/bb_labelling.h"
#include "search/flexible_astar.h"
#include "search/graph_expansion_policy.h"
#include "search/problem_instance.h"
#include "search/search_node.h"
#include "search/solution.h"
#include "util/workload_manager.h"

#include <algorithm>

struct shared_data
{
    warthog::label::bb_labelling* lab_;
    warthog::util::workload_manager* workload_;
};

struct bbl_search_listener
{
    std::vector<uint32_t>* first_move;
    warthog::label::bb_labelling* lab; 
    uint32_t source_id;

    inline void
    generate_node(warthog::search_node* from, 
                  warthog::search_node* succ, 
                  warthog::cost_t edge_cost,
                  uint32_t edge_id) 
    {
        if(from == 0) { return; } // start node 

        if((uint32_t)from->get_id() == source_id) // start node successors
        { 
            assert(edge_id < 
            lab->get_graph()->get_node(source_id)->out_degree());
            first_move->at((uint32_t)succ->get_id()) = edge_id; 
        }
        else // all other nodes
        {
            uint32_t s_id = (uint32_t)succ->get_id();
            uint32_t f_id = (uint32_t)from->get_id();
            double alt_g = from->get_g() + edge_cost;
            double g_val = 
                succ->get_search_number() == from->get_search_number() ? 
                succ->get_g() : warthog::INF32; 

            assert(first_move->at(f_id) < 
            lab->get_graph()->get_node(source_id)->out_degree());

            //  update first move
            if(alt_g < g_val) 
            { first_move->at(s_id) = first_move->at(f_id); }
        }
    };

    inline void
    expand_node(warthog::search_node* current)
    {
        if((uint32_t)current->get_id() == source_id) { return; }

        uint32_t node_id = (uint32_t)current->get_id();
        assert(node_id < first_move->size());

        uint32_t edge_idx = first_move->at(node_id);
        warthog::label::bb_label& s_lab = lab->get_label(source_id, edge_idx);

        int32_t x, y;
        lab->get_graph()->get_xy(node_id, x, y);
        s_lab.bbox_.grow(x, y);
    };

    inline void
    relax_node(warthog::search_node* current) { }
};

warthog::label::bb_labelling::bb_labelling(warthog::graph::xy_graph* g)
    : g_(g)
{
   
    // allocate memory for edge labels
    lab_ = new std::vector< std::vector < bb_label > >();
    lab_->resize(g_->get_num_nodes());
}

warthog::label::bb_labelling::~bb_labelling()
{ 
    delete lab_;
}

// compute labels for all nodes specified by the given workload
void
warthog::label::bb_labelling::precompute(
        warthog::util::workload_manager* workload)
{
    // The actual precompute function. We construct a 
    // Dijkstra-based preprocessing to improve the labels for 
    // selected sets of nodes
    void*(*thread_compute_fn)(void*) = 
    [] (void* args_in) -> void*
    {
        warthog::helpers::thread_params* par = 
            (warthog::helpers::thread_params*) args_in;
        shared_data* shared = (shared_data*) par->shared_;

        warthog::label::bb_labelling* lab = shared->lab_;
        warthog::util::workload_manager* workload = shared->workload_;

        // variable used to track the node currently being processed
        uint32_t source_id;
        std::vector<uint32_t> first_move(lab->get_graph()->get_num_nodes());

        warthog::zero_heuristic h;
        warthog::pqueue_min open;
        bbl_search_listener listener;
        warthog::simple_graph_expansion_policy expander(shared->lab_->g_);

        warthog::flexible_astar 
            <warthog::zero_heuristic, 
            warthog::simple_graph_expansion_policy,
            warthog::pqueue_min,
            bbl_search_listener>
                dijk(&h, &expander, &open, &listener);

        listener.first_move = &first_move;
        listener.lab = lab;
        for(uint32_t i = 0; i < lab->get_graph()->get_num_nodes(); i++)
        {
            // skip any nodes not part of the precomputation workload
            if(!workload->get_flag(i))
            { continue; }

            // source nodes are evenly divided among all threads;
            // skip any source nodes not intended for current thread
            if((i % par->max_threads_) != par->thread_id_) 
            { continue; }

            source_id = i;
            listener.source_id = source_id;

            uint32_t ext_source_id = 
                lab->get_graph()->to_external_id(source_id);
            warthog::problem_instance problem(ext_source_id,warthog::SN_ID_MAX);
            //problem.verbose_ = true;
            warthog::solution sol;
            dijk.get_path(problem, sol);
            par->nprocessed_++;
        }
        return 0;
    };

    warthog::timer t;
    t.start();

    shared_data shared;
    shared.lab_ = this;
    shared.workload_ = workload;
    
    // every egde gets allocated a (dummy, invalid) label
    for(uint32_t n_id = 0; n_id < g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = this->g_->get_node(n_id);
        lab_->at(n_id).resize(n->out_degree());
    }   

    std::cerr << "computing dijkstra labels\n";
    warthog::helpers::parallel_compute(
            thread_compute_fn, &shared, 
            workload->num_flags_set());
    t.stop();
    std::cerr << "done. time " << t.elapsed_time_nano() / 1e9 << " s\n";
}

std::istream&
warthog::label::operator>>(std::istream& in, warthog::label::bb_label& label)
{
    in >> label.bbox_;
    return in;
}

std::ostream&
warthog::label::operator<<(std::ostream& out, warthog::label::bb_label& label)
{
    out << label.bbox_;
    return out;
}

std::istream&
warthog::label::operator>>(std::istream& in, warthog::label::bb_labelling& lab)
{
    for(uint32_t n_id = 0; n_id < lab.g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = lab.g_->get_node(n_id);
        warthog::graph::edge_iter begin = n->outgoing_begin();
        //warthog::graph::edge_iter end = n->outgoing_end();
        for(    warthog::graph::edge_iter it = begin; 
                in.peek() != ';'; 
                it++)
        {
            bb_label bbox;
            in >> bbox; 
            lab.lab_->at(n_id).push_back(bbox);

            if(!in.good())
            {
                std::cerr << "unexpected error while reading labels\n";
                std::cerr 
                    << "[debug info] node: " << n_id
                    << " out-edge-index: " << (it - begin) << "\n";
                return in;
            }
        }
        in.get(); // eat the terminator character ';'
        assert(lab.lab_->at(n_id).size() == n->out_degree());
    }
    return in;
}

std::ostream&
warthog::label::operator<<(std::ostream& out,warthog::label::bb_labelling& lab)
{
    for(uint32_t n_id = 0; n_id < lab.g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = lab.g_->get_node(n_id);
        warthog::graph::edge_iter begin = n->outgoing_begin();
        warthog::graph::edge_iter end = n->outgoing_end();
        assert(n->out_degree() == lab.lab_->at(n_id).size());
        for(warthog::graph::edge_iter it = begin; it != end; it++)
        {
            out << lab.lab_->at(n_id).at((size_t)(it - begin));
            if(!out.good())
            {
                std::cerr << "unexpected error while writing labels\n";
                std::cerr 
                    << "[debug info] node: " << n_id
                    << " out-edge-index: " << (it-begin) << "\n";
                return out;
            }
        }
        out << ';'; // terminator to indicate end-of-label-set for node n
    }
    return out;
}

