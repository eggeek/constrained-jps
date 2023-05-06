// cpd/graph_oracle.h
//
// A Compressed Path Database is an oracle that tells
// the first move on the optimal path: from any source
// node in graph V, to any other node, in graph V. 
// The implementation is based on descriptions from 
// the following paper:
//
// [A. Botea, B. Strasser and D. Harabor. 2015. 
//  Compressing Optimal Paths with Run Length Encoding.
//  Journal of Artificial Intelligence Research (JAIR)]
//
//
// @author: dharabor
// @created: 2020-02-26
//

#ifndef WARTHOG_CPD_GRAPH_ORACLE_H
#define WARTHOG_CPD_GRAPH_ORACLE_H

#include "binary.h"
#include "constants.h"
#include "cpd.h"
#include "geography.h"
#include "graph.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"

namespace warthog
{

namespace cpd
{

enum symbol {FORWARD, REVERSE, BEARING, TABLE, REV_TABLE};

template<symbol T>
class graph_oracle_base
{
    public: 
        graph_oracle_base(warthog::graph::xy_graph* g)
             : g_(g), div_(1), mod_(0), offset_(0)
        {
            order_.resize(g_->get_num_nodes());
            fm_.resize(g_->get_num_nodes());
        }

        graph_oracle_base() : g_(nullptr) { }

        virtual ~graph_oracle_base() { }

        graph_oracle_base(const graph_oracle_base&) = default;

        bool
        operator==(const graph_oracle_base& other)
        {
            if (order_ != other.order_)
            {
                return false;
            }

            if (fm_.size() != other.fm_.size())
            {
                return false;
            }

            for (size_t i = 0; i < fm_.size(); i++)
            {
                std::vector<warthog::cpd::rle_run32> row1 = fm_.at(i);
                std::vector<warthog::cpd::rle_run32> row2 = other.fm_.at(i);

                if (row1.size() != row2.size())
                {
                    return false;
                }

                for (size_t j = 0; j < row1.size(); j++)
                {
                    if (row1.at(j).data_ != row2.at(j).data_)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        inline uint32_t 
        get_move(warthog::sn_id_t source_id, 
                 warthog::sn_id_t target_id)
        // This should really be an error
        { return warthog::cpd::CPD_FM_NONE; }

        inline void
        clear()
        {
            order_.clear();
            fm_.clear();
        }

        inline void
        compute_dfs_preorder(uint32_t seed=0)
        {
            warthog::cpd::compute_dfs_preorder(g_, &order_);
        }

        // convert the column order into a map: from vertex id to its ordered
        // index
        inline void
        value_index_swap_array()
        {
            warthog::helpers::value_index_swap_array(order_);
        }

        // compress a given first-move table @param row and associate
        // the compressed result with source node @param source_id
        void
        add_row(uint32_t source_id, std::vector<warthog::cpd::fm_coll>& row)
        {
            // source gets a wildcard move
            row.at(source_id) = warthog::cpd::CPD_FM_NONE;

            // greedily compress the row w.r.t. the current column order
            warthog::cpd::fm_coll moveset = row.at(order_.at(0));
            uint32_t head = 0;
            for(uint32_t index = 0; index < row.size(); index++)
            {
                assert(moveset > 0);
                if((moveset & row.at(order_.at(index))) == 0)
                {
                    uint32_t firstmove = __builtin_ffsl(moveset) - 1;
                    assert(firstmove < warthog::cpd::CPD_FM_MAX);
                    fm_.at(source_id).push_back(
                            warthog::cpd::rle_run32{ (head << 4) | firstmove} );
                    moveset = row.at(order_.at(index));
                    head = index;
                }
                moveset = moveset & row.at(order_.at(index));
            }

            // add the last run
            uint32_t firstmove = __builtin_ffsl(moveset) - 1;
            assert(firstmove < warthog::cpd::CPD_FM_MAX);
            fm_.at(source_id).push_back(
                    warthog::cpd::rle_run32{ (head << 4) | firstmove} );

        //    std::cerr << "compressed source row " << source_id << " with "
        //        << fm_.at(source_id).size() << std::endl;
        }

        inline warthog::graph::xy_graph* 
        get_graph() { return g_; } 

        inline void
        set_graph(warthog::graph::xy_graph* g)
        { g_ = g; }

        inline size_t
        mem()
        {
            size_t retval = 
                g_->mem() + 
                sizeof(uint32_t) * order_.size() + 
                sizeof(std::vector<warthog::cpd::rle_run32>) * fm_.size();

            for(uint32_t i = 0; i < fm_.size(); i++)
            {
                retval += sizeof(warthog::cpd::rle_run32) * fm_.at(i).size();
            }

            return retval; 
        }

        friend std::ostream&
        operator<<(std::ostream& out, graph_oracle_base& lab)
        {
            warthog::timer mytimer;
            mytimer.start();

            // write graph size
            uint32_t num_nodes = lab.g_->get_num_nodes();
            out.write((char*)(&num_nodes), 4);

            // write node ordering
            assert(lab.order_.size() == num_nodes);
            for(uint32_t i = 0; i < num_nodes; i++)
            {
                uint32_t n_id = lab.order_.at(i);
                out.write((char*)(&n_id), 4);
            }

            // write the runs for each row
            uint32_t row_count = 0;
            uint32_t run_count = 0;
            for(uint32_t row_id = 0; row_id < lab.g_->get_num_nodes(); row_id++)
            {
                // write the number of runs
                uint32_t num_runs = (uint32_t)lab.fm_.at(row_id).size();
                // Skip empty runs
                if (num_runs == 0) { continue; }

                out.write((char*)(&num_runs), 4);
                row_count++;

                for(uint32_t run = 0; run < num_runs; run++)
                {
                    out << lab.fm_.at(row_id).at(run);
                    run_count++;
                    if(!out.good())
                    {
                        std::cerr << "err; while writing labels\n";
                        std::cerr
                            << "[debug info] "
                            << " row_id " << row_id
                            << " run# " << lab.fm_.at(row_id).size()
                            << ". aborting.\n";
                        return out;
                    }
                }

            }
            mytimer.stop();

            std::cerr
                << "wrote to disk " << row_count
                << " rows and "
                << run_count << " runs. "
                << " time: " << (double)mytimer.elapsed_time_nano() / 1e9
                << " s \n";
            return out;
        }

        friend std::istream&
        operator>>(std::istream& in, graph_oracle_base& lab)
        {
            // read the graph size data
            warthog::timer mytimer;
            mytimer.start();

            uint32_t num_nodes;
            in.read((char*)(&num_nodes), 4);
            // Need to check whether we have initialized the graph as
            // serialising removes the internal pointer.
            if(lab.g_ != nullptr && num_nodes != lab.g_->get_num_nodes())
            {
                std::cerr
                    << "err; " << "input mismatch. cpd file says " << num_nodes
                    << " nodes, but graph contains " << lab.g_->get_num_nodes()
                    << "\n";
                return in;
            }

            lab.fm_.clear();
            lab.order_.resize(num_nodes);

            // read the vertex-to-column-order mapping
            for(uint32_t i = 0; i < num_nodes; i++)
            {
                uint32_t n_id;
                in.read((char*)(&n_id), 4);
                lab.order_.at(i) = n_id;
            }

            // read the RLE data
            uint32_t run_count = 0;
            lab.fm_.resize(num_nodes);
            for(uint32_t row_id = 0; row_id < num_nodes; row_id++)
            {
                // Check if we have a partial CPD file
                if (in.peek() == EOF)
                {
                    lab.fm_.resize(row_id);
                    std::cerr << "early stop; ";
                    break;
                }
                // number of runs for this row
                uint32_t num_runs;
                in.read((char*)(&num_runs), 4);

                // read all the runs for the current row
                for(uint32_t i = 0; i < num_runs; i++)
                {
                    warthog::cpd::rle_run32 tmp;
                    in >> tmp;
                    lab.fm_.at(row_id).push_back(tmp);
                    run_count++;

                    if(!in.good())
                    {
                        std::cerr << "err; while reading firstmove labels\n";
                        std::cerr
                            << "[debug info] "
                            << " row_id " << row_id
                            << " run# " << i << " of " << lab.fm_.size()
                            << ". aborting.\n";
                        return in;
                    }
                }
            }
            mytimer.stop();

            std::cerr
                << "read from disk " << lab.fm_.size()
                << " rows and "
                << run_count << " runs. "
                << " time: " << (double)mytimer.elapsed_time_nano() / 1e9
                << " s\n";
            return in;
        }

        /**
         * Append operator for CPDs. Used when building partial CPDs so we can
         * join them into a single one.
         */
        void
        append_fm(const graph_oracle_base &cpd)
        {
            fm_.insert(fm_.end(), cpd.fm_.begin(), cpd.fm_.end());
        }

        void
        compute_row(uint32_t source_id, warthog::search* dijk,
                    std::vector<warthog::cpd::fm_coll> &s_row)
        {
            warthog::problem_instance problem(source_id);
            warthog::solution sol;

            std::fill(s_row.begin(), s_row.end(), warthog::cpd::CPD_FM_NONE);
            dijk->get_path(problem, sol);
            add_row(source_id, s_row);
        }

        // TODO should only be used with reverse schemes
        std::vector<warthog::cpd::rle_run32>&
        get_row(warthog::sn_id_t target_id)
        {
            size_t row_id;
            if(div_ > 1)
            {
                row_id = target_id / div_;
            }
            else if(mod_ > 0)
            {
                row_id = target_id % mod_;
            }
            else if(offset_ > 0)
            {
                row_id = target_id - offset_;
            }
            else
            {
                row_id = target_id;
            }

            assert(row_id >= 0);
            assert(row_id < fm_.size());

            return fm_.at(row_id);
        }

        void
        set_row(size_t row_id, std::vector<warthog::cpd::rle_run32>& row)
        {
            fm_.at(row_id) = row;
        }

        void
        set_div(uint32_t div)
        { div_ = div; }

        void
        set_mod(uint32_t mod)
        { mod_ = mod; }

        void
        set_offset(uint32_t offset)
        { offset_ = offset; }

    private:
        std::vector<std::vector<warthog::cpd::rle_run32>> fm_;
        std::vector<uint32_t> order_;
        warthog::graph::xy_graph* g_;
        uint32_t div_;
        uint32_t mod_;
        uint32_t offset_;
};

typedef warthog::cpd::graph_oracle_base<FORWARD> graph_oracle;

std::ostream&
operator<<(std::ostream& out, warthog::cpd::graph_oracle& o);

std::istream&
operator>>(std::istream& in, warthog::cpd::graph_oracle& o);

void
compute_row(uint32_t source_id, warthog::cpd::graph_oracle* cpd,
            warthog::search* dijk, std::vector<warthog::cpd::fm_coll> &s_row);

typedef std::function<bool(uint32_t&)> t_find_fn;

inline uint32_t
binary_find_row(uint32_t target_index,
                std::vector<warthog::cpd::rle_run32>& row)
{
    uint32_t end = (uint32_t)row.size();
    t_find_fn find_target = [&target_index, &row](uint32_t mid)
    {
        return target_index < row.at(mid).get_index();
    };

    return util::binary_find_first<uint32_t, t_find_fn>(0, end, find_target);
}

template<>
inline uint32_t
graph_oracle_base<warthog::cpd::FORWARD>::get_move(
    warthog::sn_id_t source_id, warthog::sn_id_t target_id)
{
    if(fm_.at(source_id).size() == 0) { return warthog::cpd::CPD_FM_NONE; }

    std::vector<warthog::cpd::rle_run32>& row = fm_.at(source_id);
    uint32_t target_index = order_.at(target_id);
    uint32_t begin = binary_find_row(target_index, row);

    return row.at(begin).get_move();
}

// In a reverse CPD we get the row with the target's id, and then try to find
// the first move from the source.
template<>
inline uint32_t
graph_oracle_base<warthog::cpd::REVERSE>::get_move(
    warthog::sn_id_t source_id, warthog::sn_id_t target_id)
{
    std::vector<warthog::cpd::rle_run32>& row = get_row(target_id);
    if(row.size() == 0) { return warthog::cpd::CPD_FM_NONE; }

    uint32_t target_index = order_.at(source_id);
    uint32_t begin = binary_find_row(target_index, row);

    return row.at(begin).get_move();
}

template<>
inline uint32_t
graph_oracle_base<warthog::cpd::BEARING>::get_move(
    warthog::sn_id_t source_id, warthog::sn_id_t target_id)
{

    if(fm_.at(target_id).size() == 0) { return warthog::cpd::CPD_FM_NONE; }

    std::vector<warthog::cpd::rle_run32>& row = fm_.at(target_id);
    uint32_t target_index = order_.at(source_id);
    uint32_t begin = binary_find_row(target_index, row);
    uint8_t fm = row.at(begin).get_move();

    // Check if we have a counter-/clock-wise wildcard
    if(fm < 2)
    {
        // safeguard
        fm = warthog::cpd::CPD_FM_NONE;

        int32_t xa, ya, xb, yb, xt, yt, xs, ys;
        warthog::graph::node* node = g_->get_node(source_id);
        warthog::graph::edge_iter from = node->outgoing_begin();
        g_->get_xy(target_id, xt, yt);
        g_->get_xy(source_id, xs, ys);

        for(warthog::graph::ECAP_T edge_id = 0;
            edge_id < node->out_degree();
            edge_id++)
        {
            warthog::graph::edge_iter to = from + 1;

            // Wrap around
            if (to == node->outgoing_end())
            {
                to = node->outgoing_begin();
            }

            g_->get_xy(from->node_id_, xa, ya);
            g_->get_xy(to->node_id_, xb, yb);

            if(warthog::geo::between_xy(xs, ys, xa, ya, xt, yt, xb, yb))
            {
                if(fm == warthog::cpd::CW)
                {
                    fm = edge_id + 1;

                    if(fm >= node->out_degree())
                    {
                        fm = 0;
                    }
                }
                else // if(fm == warthog::cpd::CCW)
                {
                    fm = edge_id;
                }
                break;
            }
            from = to;
        }
        assert(fm != warthog::cpd::CPD_FM_NONE);
    }
    else
    {
            fm -= 2;
    }

    return fm;
}

// Finding a first move is a lookup, a mask and a shift
inline uint32_t
get_table_move(std::vector<warthog::cpd::rle_run32>& row, uint32_t index)
{
    if(row.size() == 0) { return warthog::cpd::CPD_FM_NONE; }

    uint32_t entry = index / 8; // Which 32bit int contains the information
    uint8_t shift = (index % 8) * 4; // Where in the 32bit int is the fm
    uint32_t mask = 0xF << shift;

    // TODO look into `bextr`
    return (row.at(entry).data_ & mask) >> shift;
}

template<>
inline uint32_t
warthog::cpd::graph_oracle_base<warthog::cpd::REV_TABLE>::get_move(
    warthog::sn_id_t source_id, warthog::sn_id_t target_id)
{
    return get_table_move(get_row(target_id), order_.at(source_id));
}

template<>
inline uint32_t
warthog::cpd::graph_oracle_base<warthog::cpd::TABLE>::get_move(
    warthog::sn_id_t source_id, warthog::sn_id_t target_id)
{
    return get_table_move(get_row(source_id), order_.at(target_id));
}

// For some reason, this needs to be defined in the .cpp. But we cannot do the
// same for `get_move()`...
//
// NOTE We *need* the header declaration otherwise the template resolution will
//      always use the default.
template<>
void
warthog::cpd::graph_oracle_base<warthog::cpd::TABLE>::add_row(
    uint32_t target_id, std::vector<warthog::cpd::fm_coll>& row);

template<>
void
warthog::cpd::graph_oracle_base<warthog::cpd::REV_TABLE>::add_row(
    uint32_t target_id, std::vector<warthog::cpd::fm_coll>& row);
}

}

#endif
