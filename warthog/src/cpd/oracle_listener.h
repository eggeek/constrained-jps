#ifndef __ORACLE_LISTENER_H_
#define __ORACLE_LISTENER_H_

#include "cpd.h"
#include "graph_oracle.h"

namespace warthog
{

namespace cpd
{

class oracle_listener
{
  public:
    virtual ~oracle_listener() {};

    virtual void
    generate_node(warthog::search_node *from, warthog::search_node *succ,
                  warthog::cost_t edge_cost, uint32_t edge_id) = 0;

    inline void
    expand_node(warthog::search_node* current) { }

    inline void
    relax_node(warthog::search_node* current) { }

    void
    set_run(warthog::sn_id_t* source_id,
            std::vector<warthog::cpd::fm_coll>* s_row)
    {
        source_id_ = source_id;
        s_row_ = s_row;
    }

  protected:
    warthog::sn_id_t* source_id_;
    std::vector<warthog::cpd::fm_coll>* s_row_;
};

// helps to precompute first-move data
template<warthog::cpd::symbol SYM>
class graph_oracle_listener final : public oracle_listener
{
  public:
    graph_oracle_listener(warthog::cpd::graph_oracle_base<SYM>* oracle)
        : oracle_(oracle) {}

    inline void
    generate_node(warthog::search_node *from, warthog::search_node *succ,
                  warthog::cost_t edge_cost, uint32_t edge_id)
    {
        if(from == 0) { return; } // start node

        if(from->get_id() == *source_id_) // start node successors
        {
            //assert(s_row_.at(succ->get_id()) == 0);
            assert(edge_id <
                        oracle_->get_graph()->get_node(
                        (uint32_t)*source_id_)->out_degree());
            s_row_->at(succ->get_id()) = (1 << edge_id);
            assert(s_row_->at(succ->get_id()));
        }
        else // all other nodes
        {
            warthog::sn_id_t succ_id = succ->get_id();
            warthog::sn_id_t from_id = from->get_id();
            double alt_g = from->get_g() + edge_cost;
            double g_val =
                succ->get_search_number() == from->get_search_number() ?
                succ->get_g() : DBL_MAX;

            //  update first move
            if(alt_g < g_val)
            {
                s_row_->at(succ_id) = s_row_->at(from_id);
                assert(s_row_->at(succ_id) == s_row_->at(from_id));
            }

            // add to the list of optimal first moves
            if(alt_g == g_val)
            {
                s_row_->at(succ_id) |= s_row_->at(from_id);
                assert(s_row_->at(succ_id) >= s_row_->at(from_id));
            }

        }
    }

  private:
    warthog::cpd::graph_oracle_base<SYM>* oracle_;
};

// helps to precompute first-move data, this time we build the rows in reverse.
template<warthog::cpd::symbol SYM>
class reverse_oracle_listener final : public oracle_listener
{
  public:
    reverse_oracle_listener(
        warthog::cpd::graph_oracle_base<SYM>* oracle)
        : oracle_(oracle) {}

    inline void
    generate_node(warthog::search_node *from, warthog::search_node *succ,
                  warthog::cost_t edge_cost, uint32_t edge_id)
    {
        if(from == nullptr) { return; } // start node has no predecessor

        double alt_g = from->get_g() + edge_cost;
        double g_val =
            succ->get_search_number() == from->get_search_number() ?
            succ->get_g() : DBL_MAX;
        // We record the optimal move towards a node which is the id of the
        // predecessor's edge
        graph::node* pred = oracle_->get_graph()->get_node(succ->get_id());
        graph::edge_iter eit = pred->find_edge(from->get_id());
        warthog::cpd::fm_coll fm = 1 << (eit - pred->outgoing_begin());

        assert(eit != pred->outgoing_end());

        //  update first move
        if(alt_g < g_val)
        {
            // Pointer arithmetic ftw
            s_row_->at(succ->get_id()) = fm;
            assert(s_row_->at(succ->get_id()));
        }

        // add to the list of optimal first moves
        if(alt_g == g_val)
        {
            s_row_->at(succ->get_id()) |= fm;
        }
    }

  private:
    warthog::cpd::graph_oracle_base<SYM>* oracle_;
};

// helps to precompute first-move data, this one does bearing compression on
// road networks. This may not work on classic gridmaps.
//
// TODO add a forward listener with orientation
class reverse_bearing_oracle_listener final : public oracle_listener
{
  public:
    reverse_bearing_oracle_listener(
        warthog::cpd::graph_oracle_base<warthog::cpd::BEARING>* oracle)
        : oracle_(oracle) {}

    inline void
    generate_node(warthog::search_node *from, warthog::search_node *succ,
                  warthog::cost_t edge_cost, uint32_t edge_id)
    {
        if(from == nullptr) { return; } // start node has no predecessor

        double alt_g = from->get_g() + edge_cost;
        double g_val =
            succ->get_search_number() == from->get_search_number() ?
            succ->get_g() : DBL_MAX;
        graph::node* pred = oracle_->get_graph()->get_node(succ->get_id());
        graph::edge_iter eit = pred->find_edge(from->get_id());

        assert(eit != pred->outgoing_end());
        assert(
            (eit - pred->outgoing_begin()) < (warthog::cpd::CPD_FM_MAX - 2));
        // Offset the first move symbol by two we can fit the clockwise symbols.
        warthog::cpd::fm_coll fm = 1 << ((eit - pred->outgoing_begin()) + 2);

        // Circle back around on both sides
        warthog::graph::edge_iter prev = eit - 1;
        warthog::graph::edge_iter next = eit + 1;
        if(eit == pred->outgoing_begin())
        {
            prev = pred->outgoing_end() - 1;
        }

        if(next == pred->outgoing_end())
        {
            next = pred->outgoing_begin();
        }

        // Next, find whether it is (counter-) clockwise from the target
        warthog::graph::xy_graph* g = oracle_->get_graph();
        int32_t xf, yf, xs, ys, xt, yt, xp, yp, xn, yn;
        g->get_xy(from->get_id(), xf, yf);
        g->get_xy(succ->get_id(), xs, ys);
        g->get_xy(*source_id_, xt, yt);
        g->get_xy(prev->node_id_, xp, yp);
        g->get_xy(next->node_id_, xn, yn);

        // To add a wildcard, we need the target's bearing to lie between the
        // first move and one of its neighbours. We record which direction to
        // rotate from that bearing to find the first move.
        if(warthog::geo::between_xy(xs, ys, xp, yp, xt, yt, xf, yf))
        {
            fm |= 1 << warthog::cpd::CW;
        }
        if(warthog::geo::between_xy(xs, ys, xf, yf, xt, yt, xn, yn))
        {
            fm |= 1 << warthog::cpd::CCW;
        }

        //  update first move
        if(alt_g < g_val)
        {
            s_row_->at(succ->get_id()) = fm;
            assert(s_row_->at(succ->get_id()));
        }

        // add to the list of optimal first moves
        if(alt_g == g_val)
        {
            s_row_->at(succ->get_id()) |= fm;
        }
    }

  private:
    warthog::cpd::graph_oracle_base<warthog::cpd::BEARING>* oracle_;
};

}

}

#endif // __ORACLE_LISTENER_H_
