#include "apriori_filter.h"
#include "euclidean_heuristic.h"
#include "flexible_astar.h"
#include "graph_expansion_policy.h"
#include "heap.h"
#include "helpers.h"
#include "lazy_graph_contraction.h"
#include "zero_heuristic.h"

#include <algorithm>
#include <functional>
#include <utility>

warthog::ch::lazy_graph_contraction::lazy_graph_contraction()
    : verbose_(false) { }

bool
warthog::ch::operator<(const ch_pair& first, const ch_pair& second)
{
    return first.cval_ < second.cval_;
}

bool
warthog::ch::lazy_graph_contraction::contract(
        warthog::ch::ch_data* ret,
        bool verify_priorities, uint32_t c_pct)
{
    warthog::timer mytimer;
    double t_begin = mytimer.get_time_micro();
    std::cerr << "contracting graph " << ret->g_->get_filename() << std::endl;

    ret->type_ = warthog::ch::ch_type::UP_DOWN;

    preliminaries(ret);
    if(c_pct < 100)
    { std::cerr << "partial contraction " << "(first "<<c_pct<<"% of nodes only)\n"; }
    uint32_t edges_before = ret->g_->get_num_edges_out();

    // contract nodes in the order produced by ::next
    uint32_t total_nodes = (uint32_t)((ret->g_->get_num_nodes()*c_pct) / 100);

    double t_last = mytimer.get_time_micro();
    while(true)
    {
        // contract the highest priority node  +
        // record some search effort metrics
        uint64_t num_expansions = total_expansions_;
        uint64_t num_searches = total_searches_;
        uint64_t num_lazy = total_lazy_updates_;
        mytimer.start();

        uint32_t best_id = next(verify_priorities, c_pct);
        if(best_id == warthog::INF32) { break; }

        if(this->verbose_)
        {
            std::cerr << "contracting " << best_id << std::endl;
        }

        uc_neis_.clear();
        in_shorts.clear();
        out_shorts.clear();
        terms_[best_id] = contract_node(best_id, false);

        for(auto uc_iter : uc_neis_)
        {
            uint32_t neighbour_id = uc_iter;
            if(!u_filter_->get_flag(neighbour_id)) { continue; } // updated

            // re-compute importance metrics
            niv_metrics niv =
            contract_node(neighbour_id, true);
            total_lazy_updates_++;

            // update the "search space size" estimate
            niv.depth_ = std::max(niv.depth_, terms_[best_id].depth_+1);

            // recompute priority and update heap
            int32_t cval = compute_contraction_priority(niv);
            if(cval < hn_pool_[neighbour_id].get_element().cval_)
            {
               hn_pool_[neighbour_id].get_element().cval_ = cval;
               heap_->decrease_key(&hn_pool_[neighbour_id]);
            }
            else
            {
                hn_pool_[neighbour_id].get_element().cval_ = cval;
                heap_->increase_key(&hn_pool_[neighbour_id]);
            }

            // store the metrics
            terms_[neighbour_id] = niv;
            u_filter_->set_flag_false(neighbour_id);
        }
        mytimer.stop();

        num_expansions = total_expansions_ - num_expansions;
        num_searches = total_searches_ - num_searches;
        num_lazy = total_lazy_updates_ - num_lazy;

        //if((mytimer.get_time_micro() - t_last) > 1000000)
        //{
            t_last = mytimer.get_time_micro();
            std::cerr
                //<< "\r "
                //<< (int)((order_.size() / (double)g_->get_num_nodes()) * 100)
                << "time: " << (int)((t_last - t_begin)/1000000) << "s"
                << "; " << order_->size() << " /  " << total_nodes
                << "; nid: " << best_id
                << ", pri: " << hn_pool_[best_id].get_element().cval_
                << "; #out: " << ret->g_->get_node(best_id)->out_degree()
                << "; #witn " << num_searches
                << "; #lazy: " << num_lazy
                << "; #exps: " << num_expansions
                << "; micros: " << (int32_t)mytimer.elapsed_time_micro();
            terms_[best_id].print(std::cerr);
            std::cerr << std::endl;
        //}
    }

    // until now we kept track of the _order_ in which nodes were
    // contracted. but for online search we need to know each node's _level_
    // here we convert from one to the other
    warthog::helpers::value_index_swap_array(*ret->level_);

    // sort successors in descending contraction order
    // (the highest-level successor appears first)
    warthog::ch::sort_successors(ret);

    // cleanup
    postliminaries();

    std::cerr
        << "\ngraph, contracted. " << std::endl
        << "time "
        << ((double)(mytimer.get_time_micro() - t_begin)) / 1e6 << " (s)"
        << "; edges before " << edges_before
        << "; edges after " << g_->get_num_edges_out() << std::endl
        << "total searches " << total_searches_
        << "; total expansions (x 1e6) " << ((double)total_expansions_)/1e6
        << std::endl
        << "lazy updates "<< total_lazy_updates_
        << std::endl;
    return true;
}

void
warthog::ch::lazy_graph_contraction::preliminaries(warthog::ch::ch_data* chd)
{
    order_ = chd->level_;
    g_ = chd->g_;
    //heuristic_ = new warthog::euclidean_heuristic(g_);
    heuristic_ = new warthog::zero_heuristic();
    c_filter_ = new warthog::apriori_filter(g_);
    u_filter_ = new warthog::apriori_filter(g_);
    fexpander_ = new warthog::graph_expansion_policy<warthog::apriori_filter>(
                g_, c_filter_);
    bexpander_ = new warthog::graph_expansion_policy<warthog::apriori_filter>(
                g_, c_filter_);

    //alg_ = new bidirectional_search< warthog::euclidean_heuristic,
    alg_ = new bidirectional_search< warthog::zero_heuristic,
                 warthog::graph_expansion_policy<warthog::apriori_filter>>(
                         fexpander_, bexpander_, heuristic_);

//    open_ = new warthog::pqueue_min();
//    expander_ = new warthog::graph_expansion_policy< warthog::apriori_filter >
//        (g_, c_filter_);
//    alg2_ = new warthog::flexible_astar<
//                    warthog::zero_heuristic,
//                    warthog::graph_expansion_policy<warthog::apriori_filter>,
//                    warthog::pqueue_min>
//                        (heuristic_, expander_, open_);
//

    size_t sz_g = g_->get_num_nodes();
    heap_ = new warthog::heap<ch_pair>((uint32_t)sz_g, true);
    hn_pool_ = new heap_node<ch_pair>[sz_g];

    terms_ = new niv_metrics[sz_g];
    ws_max_expansions_ = warthog::INF32;

    // initialise edge labels with hop numbers (initially, all 1)
    for(uint32_t i = 0; i < g_->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g_->get_node(i);
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            warthog::graph::edge& out = *(n->outgoing_begin() + j);
            out.label_ = 1;
        }
        for(uint32_t j = 0; j < n->in_degree(); j++)
        {
            warthog::graph::edge& in = *(n->incoming_begin() + j);
            in.label_ = 1;
        }
    }

    // create an initial ordering by performing on each node
    // a faux contraction operation
    std::cerr << "creating initial contraction order:\n";
    total_searches_ = 0;
    total_expansions_ = 0;
    total_lazy_updates_ = 0;

    double pct_counter=0;
    for(uint32_t i = 0;
        i < g_->get_num_nodes(); i++)
    {
        if( (i / (double)g_->get_num_nodes()) >= pct_counter)
        {
            std::cerr << pct_counter*100 << "%...";
            pct_counter+=0.1;
        }
        niv_metrics niv = contract_node(i, true);
        int32_t priority = compute_contraction_priority(niv);
        hn_pool_[i] = heap_node<ch_pair>(ch_pair(i, priority));
        heap_->push(&hn_pool_[i]);
        terms_[i] = niv;
    }
    std::cerr << "100%" << std::endl;
}

void
warthog::ch::lazy_graph_contraction::postliminaries()
{
    delete [] terms_;
    terms_ = 0;

    delete [] hn_pool_;
    hn_pool_ = 0;

    delete heap_;
    heap_ = 0;

    delete alg_;
    alg_ = 0;

    delete c_filter_;
    c_filter_ = 0;

    delete u_filter_;
    u_filter_ = 0;

    delete heuristic_;
    heuristic_ = 0;

    delete fexpander_;
    fexpander_ = 0;

    delete bexpander_;
    bexpander_ = 0;

}

// identifies the node that should be contracted next NB: nodes are ranked in a
// lazy way; an initial priority is generated (lower is better) but priorities
// are not updated until a node is popped off the heap.  at this stage we
// re-compute the node priority and possibly insert it back into the heap if
// it is no longer the ``best'' candidate
//
// @return the next node id or warthog::INF32 if all nodes have been contracted
uint32_t
warthog::ch::lazy_graph_contraction::next(
        bool verify_priorities, uint32_t c_pct)
{
    // early abort; all nodes contracted
    if(heap_->size() == 0)
    {
        return warthog::INF32;
    }

    // early abort; partial contraction limit reached
    uint32_t pct = (uint32_t)((order_->size() / (warthog::cost_t)g_->get_num_nodes()) * 100);
    if(pct >= c_pct)
    {
        std::cerr << "\npartial contraction finished "
                  << "(processed "<< pct << "% of all nodes)";
        return warthog::INF32;
    }

    // pop the best contraction candidate off the heap
    heap_node<ch_pair>* best = heap_->pop();
    return best->get_element().node_id_;
}

// NB: assumes the via-node is already marked as contracted
// (and will thus not be expanded)
warthog::cost_t
warthog::ch::lazy_graph_contraction::witness_search(
        uint32_t from_id, uint32_t to_id, warthog::cost_t via_len,
        bool resume)
{
    // only search for witness paths between uncontracted neighbours
    //if(c_filter_->get_flag(from_id) || c_filter_->get_flag(to_id)) { return 0; }

    alg_->set_cost_cutoff(via_len);
    alg_->set_max_expansions_cutoff(ws_max_expansions_);
    warthog::graph::xy_graph* g = this->g_;

    // need to specify start + target ids using the identifier
    // that appears in the input file
    warthog::problem_instance pi(
            g->to_external_id(from_id),
            g->to_external_id(to_id));

    sol_.reset();
    sol_.path_.clear();
    pi.extra_params_ = (void*)resume;

    // gogogo
    alg_->get_path(pi, sol_);
    total_expansions_ += sol_.nodes_expanded_;
    total_searches_++;

    //warthog::problem_instance pi2 = pi;
    //warthog::solution sol2;
    //sol2.reset();

    //alg2_->set_cost_cutoff(via_len);
    //alg2_->set_max_expansions_cutoff(ws_max_expansions_);
    //alg2_->get_pathcost(pi, sol2);

    //if(sol_.sum_of_edge_costs_ != warthog::COST_MAX &&
    //   sol2.sum_of_edge_costs_ == warthog::COST_MAX)
    //{
    //    sol2.reset();
    //    pi.verbose_ = true;
    //    pi2.verbose_ = true;
    //    alg2_->get_pathcost(pi, sol2);

    //    std::cerr << " \nbidirectional path: \n";
    //    sol_.print_path(std::cerr);
    //    std::cerr << " bidirectional path cost " << sol_.sum_of_edge_costs_
    //        << " via_len " << via_len << std::endl;
    //    sol_.reset();
    //    alg_->get_path(pi, sol_);
    //}

    //if(    sol_.sum_of_edge_costs_ == warthog::COST_MAX &&
    //       sol2.sum_of_edge_costs_ != warthog::COST_MAX)
    //{
    //    assert( sol_.nodes_expanded_ == ws_max_expansions_ || false );
    //}
    //else if(    sol2.sum_of_edge_costs_ == warthog::COST_MAX &&
    //       sol_.sum_of_edge_costs_ != warthog::COST_MAX)
    //{
    //    assert( sol2.nodes_expanded_ == ws_max_expansions_ || false );
    //}

    //std::cerr
    //    << "witn; "
    //    << pi.start_id_ << " to " << pi.target_id_
    //    << " vialen " << via_len
    //    << " witlen " << sol_.sum_of_edge_costs_
    //    << std::endl;
    return sol_.sum_of_edge_costs_;
}

// calculate the net number of edges that result from contracting
// a given node
//
// @param n: the id of the node to contract
// @param estimate: when true, witness searches are truncated after
// a fixed number of node expansions. when false, witness searches
// are run in full.
warthog::ch::lazy_graph_contraction::niv_metrics
warthog::ch::lazy_graph_contraction::contract_node(
        uint32_t node_id, bool metrics_only)
{
    niv_metrics niv;
    warthog::graph::node* n = g_->get_node(node_id);

    // make a list of all uncontracted neighbours whose priorities
    // will need to be updated due to contracting @param node_id.
    // here we also track some importance metrics related to "deleted" edges
    niv.hops_removed_ = 1; // i.e. the current node
    niv.edel_ = 1; // as per routingkit

    // witness searches (i.e. between every pair of (in, out) neighbours)
    // NB: during each witness search, we never expand the current node
    c_filter_->set_flag_true(node_id);
    ws_max_expansions_ = 500; // as recommended by RoutingKit's implementation
    //ws_max_expansions_ = (metrics_only ? 1000 : warthog::INF32); // ddh

    // mark uncontracted incoming neighbours that will need to be reprioritised
    for(uint32_t i = 0; i < n->in_degree(); i++)
    {
        warthog::graph::edge& e_in = *(n->incoming_begin()+i);
        if(c_filter_->get_flag(e_in.node_id_)) { niv.nc_++; continue; }

        if(!metrics_only)
        {
            uc_neis_.insert(e_in.node_id_);
            u_filter_->set_flag_true(e_in.node_id_);
        }
    }

    // mark uncontracted outgoing neighbours that will need to be reprioritised
    for(uint32_t j = 0; j < n->out_degree(); j++)
    {
        warthog::graph::edge& e_out = *(n->outgoing_begin()+j);
        if(c_filter_->get_flag(e_out.node_id_)) { niv.nc_++; continue; }

        if(!metrics_only)
        {
            uc_neis_.insert(e_out.node_id_);
            u_filter_->set_flag_true(e_out.node_id_);
        }
    }

    for(uint32_t i = 0; i < n->in_degree(); i++)
    {
        warthog::graph::edge& e_in = *(n->incoming_begin()+i);
        if(c_filter_->get_flag(e_in.node_id_)) { niv.nc_++; continue; }

        // mark uncontracted incoming neighbours that will need to be reprioritised
        if(!metrics_only)
        {
            uc_neis_.insert(e_in.node_id_);
            u_filter_->set_flag_true(e_in.node_id_);
        }
        niv.edel_++;
        niv.hops_removed_ += e_in.label_;

        bool resume = false;
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            warthog::graph::edge& e_out = *(n->outgoing_begin()+j);
            if(e_in.node_id_ == e_out.node_id_) { continue; }

            if(c_filter_->get_flag(e_out.node_id_)) { niv.nc_++; continue; }

            niv.edel_++;
            niv.hops_removed_ += e_in.label_;

            warthog::cost_t cost_cutoff = e_in.wt_ + e_out.wt_;
            warthog::cost_t witness_len =
                witness_search(e_in.node_id_, e_out.node_id_, cost_cutoff, resume);
            resume = true;

            // contraction will introduce a shortcut edge only if
            // the path <in, n, out> is the only shortest path
            warthog::cost_t via_len = e_in.wt_ + e_out.wt_;
            if(witness_len > via_len)
            {
                // importance metrics related to the newly added edge
                niv.eadd_++;
                niv.hops_added_ += e_in.label_ + e_out.label_;
                if(!metrics_only)
                {
                    if(this->verbose_)
                    {
                        std::cerr << "bypassing " << node_id
                        << "; " << e_in.node_id_ << " to "
                        << e_out.node_id_ << " cost " << via_len << std::endl;
                    }

                    // these shortcuts get added after performing all witness searches
                    out_shorts.push_back(
                        std::pair<warthog::graph::node*,
                            warthog::graph::edge>(
                                g_->get_node(e_in.node_id_),
                                warthog::graph::edge(
                                    e_out.node_id_,
                                    (warthog::graph::edge_cost_t)via_len,
                                    niv.hops_added_)));

                    in_shorts.push_back(
                        std::pair<warthog::graph::node*,
                            warthog::graph::edge>(
                                g_->get_node(e_out.node_id_),
                                warthog::graph::edge(
                                    e_in.node_id_,
                                    (warthog::graph::edge_cost_t)via_len,
                                    niv.hops_added_)));
                }
            }
        }
    }

    // mark uncontracted outgoing neighbours that will need to be reprioritised
    for(uint32_t j = 0; j < n->out_degree(); j++)
    {
        warthog::graph::edge& e_out = *(n->outgoing_begin()+j);
        if(c_filter_->get_flag(e_out.node_id_)) { continue; }

        if(!metrics_only)
        {
            uc_neis_.insert(e_out.node_id_);
            u_filter_->set_flag_true(e_out.node_id_);
        }
    }

    // wrap up; clear flags (if metrics-only contraction) and
    // add shortcuts (if contracting for real)
    if(metrics_only) { c_filter_->set_flag_false(node_id); }
    else
    {
        for(auto& pair : out_shorts)
        { (pair.first)->add_outgoing(pair.second); }

        for(auto& pair : in_shorts)
        { (pair.first)->add_incoming(pair.second); }

        order_->push_back(node_id);
    }

    // another metric is the level estimate for the current node
    // (at least equal to [#contractions], assuming first level is zero)
    niv.level_ = (uint32_t)order_->size();

    // finally, we carry over the search-space-size metric (updated elsewhere)
    niv.depth_ = terms_[node_id].depth_;

    return niv;
}

size_t
warthog::ch::lazy_graph_contraction::mem()
{
    return
        heap_->mem() +
        alg_->mem() +
        sizeof(*hn_pool_)*g_->get_num_nodes() +
        sizeof(this);
}

// compute the contraction value of node @param nid
// this value represents how attractive a node is to contract
// (lower is better)
//
// it is computed by taking the weighted sum of a number of
// different metrics including edge difference, number of
// contracted neighbours and the maximum depth a search must
// reach to encounter and expand @param nid.
//
// the weights are magic numbers taken from Robert Geisberger's
// 2008 thesis.
int32_t
warthog::ch::lazy_graph_contraction::compute_contraction_priority(
        niv_metrics& niv)
{
//      ** RoutingKit heuristic **
		return (int32_t)
            (1 +
            1000*niv.depth_+
            (1000*niv.eadd_) / (1+niv.edel_) +
            (1000*niv.hops_added_) / (1+niv.hops_removed_));

//        ** GEISBERGER's EDSL heuristic, for use with verify_priority**
//        return
//            (niv.eadd_ - niv.edel_)*190 +
//            niv.nc_*120 +
//            niv.depth_;
}
