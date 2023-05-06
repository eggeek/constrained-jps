#include "apriori_filter.h"
#include "constants.h"
#include "zero_heuristic.h"
#include "fixed_graph_contraction.h"
#include "flexible_astar.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"

warthog::ch::fixed_graph_contraction::fixed_graph_contraction()
//    :  heuristic_(0), filter_(0), expander_(0), open_(0), alg_(0)
{
}

warthog::ch::fixed_graph_contraction::~fixed_graph_contraction()
{
    delete alg_;
    delete filter_;
    delete heuristic_;
    //delete expander_;
    //delete open_;
    delete fexpander_;
    delete bexpander_;
}


void
warthog::ch::fixed_graph_contraction::preliminaries(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* order, uint32_t c_pct)
{
    g_ = g;
    order_ = order;
    c_pct_ = c_pct;
    assert(order_->size() <= g_->get_num_nodes());

    order_index_ = 0;

//    delete filter_;
    filter_ = new warthog::apriori_filter(get_graph());

//    delete expander_;
//    expander_ = new warthog::graph_expansion_policy< warthog::apriori_filter >
//        (get_graph(), filter_);

//    delete open_;
//    open_ = new warthog::pqueue_min();

//    delete heuristic_;
    heuristic_ = new warthog::zero_heuristic();

//    alg_ = new flexible_astar<
//                    warthog::zero_heuristic,
//                    warthog::graph_expansion_policy<warthog::apriori_filter>,
//                    warthog::pqueue_min>
//                        (heuristic_, expander_, open_);

    fexpander_ = new warthog::graph_expansion_policy<warthog::apriori_filter>(
                g_, filter_);
    bexpander_ = new warthog::graph_expansion_policy<warthog::apriori_filter>(
                g_, filter_);

    alg_ = new bidirectional_search< warthog::zero_heuristic,
                 warthog::graph_expansion_policy<warthog::apriori_filter>>(
                         fexpander_, bexpander_, heuristic_);
}

void
warthog::ch::fixed_graph_contraction::contract(
        warthog::ch::ch_data* ret, 
        std::vector<uint32_t>* order, uint32_t c_pct)
{
    ret->type_ = warthog::ch::ch_type::UP_DOWN;
    preliminaries(ret->g_, order, c_pct);

    if(c_pct_ < 100)
    {
        std::cerr << "partially "
                  << "("<<c_pct_<<"% of nodes) ";
    }

    warthog::timer mytimer;
    double t_begin = mytimer.get_time_micro();

    std::cerr << "contracting graph " << g_->get_filename() << std::endl;
    total_searches_ = 0;
    total_expansions_ = 0;

    uint32_t edges_before = (uint32_t)g_->get_num_edges_out();
    uint32_t total_nodes = (uint32_t)g_->get_num_nodes();
    uint32_t num_contractions = 0;
    double t_last = mytimer.get_time_micro();
    for(uint32_t cid = next(); cid != warthog::INF32; cid = next())
    {
        
        uint32_t pct = (uint32_t)((num_contractions / (double)g_->get_num_nodes()) * 100);
        if(pct >= c_pct_)
        { 
            std::cerr << "\npartial contraction finished " 
                      << "(processed "<< pct << "% of all nodes)";
            break; 
        }

        // contract a node
        warthog::graph::node* n = g_->get_node(cid);
        filter_->set_flag_true(cid);
        in_shorts.clear();
        out_shorts.clear();

        if(this->verbose_)
        {
            std::cerr << "contracting " << cid << std::endl;
        }

        
        // maximum expansions per Dijkstra witness search
        // the value 500 comes from RoutingKit's CH implementation
        uint32_t max_expand = 500;

        // establish a cost upperbound on the length of witness paths
        double  max_outgoing_wt = 0;
        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge& e_out = *(n->outgoing_begin()+i);
            if(e_out.wt_ > max_outgoing_wt) { max_outgoing_wt = e_out.wt_; }
        }

        uint32_t eadd = 0;
        for(uint32_t i = 0; i < n->in_degree(); i++)
        {
            warthog::graph::edge& e_in = *(n->incoming_begin() + i);
            if(filter_->get_flag(e_in.node_id_)) { continue; }

            double max_cost = e_in.wt_ + max_outgoing_wt;
            //witness_search(e_in.node_id_, warthog::INF32, max_cost, max_expand);

            for(uint32_t j = 0; j < n->out_degree(); j++)
            {
                warthog::graph::edge& e_out = *(n->outgoing_begin()+j);
                if(e_in.node_id_ == e_out.node_id_) { continue; }
                if(filter_->get_flag(e_out.node_id_)) { continue; }

                double witness_len = 
                    witness_search(e_in.node_id_,  e_out.node_id_, max_cost, max_expand);
                //warthog::search_node* nei = 
                //     alg_->get_generated_node(e_out.node_id_);
                //double witness_len = nei ? nei->get_g() : warthog::INF32;
                double via_len = e_in.wt_ + e_out.wt_;

                if(witness_len > via_len)
                {
                    if(this->verbose_)
                    {
                        std::cerr << "bypassing " << cid 
                        << "; " << e_in.node_id_ << " to "
                        << e_out.node_id_ 
                        << "; bypass cost " << witness_len 
                        << " via cost " << via_len << std::endl;
                    }
                    eadd++;

                    out_shorts.push_back(
                        std::pair<warthog::graph::node*,
                            warthog::graph::edge>(
                                g_->get_node(e_in.node_id_),
                                warthog::graph::edge(
                                    e_out.node_id_,
                                    (warthog::graph::edge_cost_t)via_len)));

                    in_shorts.push_back(
                        std::pair<warthog::graph::node*,
                            warthog::graph::edge>(
                                g_->get_node(e_out.node_id_),
                                warthog::graph::edge(
                                    e_in.node_id_,
                                    (warthog::graph::edge_cost_t)via_len)));
                }
            }
        }

        // add the shortcuts
        for(auto& pair : out_shorts)
        { (pair.first)->add_outgoing(pair.second); }
        for(auto& pair : in_shorts)
        { (pair.first)->add_incoming(pair.second); }

        num_contractions++;

        if((mytimer.get_time_micro() - t_last) > 1000000)
        {
            std::cerr 
                << "time: " << (int)((t_last - t_begin)/1000000) << "s; "
                << num_contractions 
                << " /  " << total_nodes
                << "; current: " << cid 
                << " in: " << n->in_degree() << " out: " << n->out_degree() 
                << " eadd " << eadd << std::endl;
                std::cerr << std::flush;
                t_last = mytimer.get_time_micro();
        }
    }

    *ret->level_ = *order;
    warthog::helpers::value_index_swap_array(*ret->level_);

    warthog::ch::sort_successors(ret);

    std::cerr 
        << "\ngraph, contracted. " << std::endl
        << "time " << 
        (mytimer.get_time_micro() - t_begin) / 1000000.0 << " (s)"
        << "; edges before " << edges_before 
        << "; edges after " << g_->get_num_edges_out() << std::endl
        << "total searches " << total_searches_ 
        << "; total expansions (x 1e6) "<< ((double)total_expansions_)/1e6
        << std::endl;
}

uint32_t 
warthog::ch::fixed_graph_contraction::next()
{
    if(order_index_ < order_->size())
    {
        return order_->at(order_index_++);
    }
    return warthog::INF32;
}

// NB: assumes the via-node is already contracted
double
warthog::ch::fixed_graph_contraction::witness_search(
        uint32_t from_id, uint32_t to_id, double via_len,
        uint32_t max_expand)
{
    // pathfinding queries must specify an external start and target id
    // (i.e. as they appear in the input file)
    warthog::graph::xy_graph* g = this->get_graph();
    warthog::sn_id_t ext_from_id = g->to_external_id(from_id);
    warthog::sn_id_t ext_to_id = g->to_external_id(to_id);
    if(ext_to_id == warthog::INF32)
    {
        // hacktacular; these uint32_t ids need to be converted to 64bit sn_id_t
        ext_to_id = warthog::SN_ID_MAX;
    }

    // run the search
    alg_->set_cost_cutoff(via_len);
    alg_->set_max_expansions_cutoff(max_expand);
    warthog::problem_instance pi(ext_from_id, ext_to_id);
    warthog::solution sol;
    alg_->get_pathcost(pi, sol);

    // metrics
    total_expansions_ += sol.nodes_expanded_;
    total_searches_++;
    return sol.sum_of_edge_costs_;
}

size_t
warthog::ch::fixed_graph_contraction::mem()
{
    return 
        alg_->mem() + 
        sizeof(this);
}
