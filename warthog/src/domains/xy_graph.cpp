#include "domains/xy_graph.h"
#include "util/timer.h"

void
warthog::graph::gridmap_to_xy_graph(
    warthog::gridmap* gm, warthog::graph::xy_graph* g,
    bool store_incoming)
{
    assert(gm && g && g->get_num_nodes() == 0);
    g->clear();
    g->set_filename(gm->filename());
    g->grow(gm->get_num_traversable_tiles());

    // add each traversable tile as a node in the graph
    std::vector<uint32_t> id_map(gm->header_height() * gm->header_width());
    warthog::gridmap_expansion_policy exp(gm);
    uint32_t next_graph_id = 0;
    for(uint32_t y = 0; y < gm->header_height(); y++)
    {
        for(uint32_t x = 0; x < gm->header_width(); x++)
        {
            // we differentiate between external grid ids
            // (unpadded) and internal grid ids (with padding)
            uint32_t from_gm_id = y * gm->header_width() + x;

            // skip obstacles
            if(!gm->get_label(gm->to_padded_id(from_gm_id)))
            { continue; }

            // add graph node (we scale up all costs and coordinates)
            id_map.at(from_gm_id) = next_graph_id;
            g->set_xy(next_graph_id,
                (int32_t)(x * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR),
                (int32_t)(y * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR));
            next_graph_id++;
        }
    }

    // add edges
    for(uint32_t y = 0; y < gm->header_height(); y++)
    {
        for(uint32_t x = 0; x < gm->header_width(); x++)
        {
            // we use a grid-expansion policy here which enforces
            // standard constraints on grid moves re; e.g. corner cutting
            uint32_t from_graph_id;
            {
                from_graph_id = id_map.at(y*gm->header_width() + x);
                assert(from_graph_id < next_graph_id);
            }

            warthog::search_node* nei = 0;
            warthog::search_node* n = 0;
            edge_cost_t edge_cost = 0;

            n = exp.generate(gm->to_padded_id(y*gm->header_width() + x));
            exp.expand(n, 0);
            for(exp.first(nei, edge_cost); nei != 0; exp.next(nei, edge_cost))
            {
                uint32_t to_graph_id;
                {
                    uint32_t nei_x, nei_y;
                    gm->to_unpadded_xy((uint32_t)nei->get_id(), nei_x, nei_y);
                    uint32_t to_gm_id = nei_y * gm->header_width() + nei_x;
                    to_graph_id = id_map.at(to_gm_id);
                    assert(from_graph_id != to_graph_id);
                    assert(to_graph_id < next_graph_id);
                }

                warthog::graph::node* gr_from = g->get_node(from_graph_id);
                edge_cost_t gr_weight = edge_cost * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
                gr_from->add_outgoing(warthog::graph::edge(to_graph_id, gr_weight));
                if(store_incoming)
                {
                    warthog::graph::node* gr_to = g->get_node(to_graph_id);
                    gr_to->add_incoming(warthog::graph::edge(from_graph_id, gr_weight));
                }
            }
        }
    }
}

void
warthog::graph::dimacs_to_xy_graph(
        warthog::dimacs_parser& dimacs, warthog::graph::xy_graph& g,
        bool reverse_arcs,
        bool store_incoming_edges, bool enforce_euclidean)
{
    g.clear();
    g.set_filename(dimacs.get_gr_filename().c_str());

   // xy_graph uses 0-indexed ids, DIMACS uses 1-indexed ids
   // we use an offset to convert between the two schemes
    uint32_t offset  = 1;

    // allocate memory for nodes
    uint32_t num_nodes_dimacs = dimacs.get_num_nodes();
    g.grow(num_nodes_dimacs);

    // set xy coordinates of each node
    for(warthog::dimacs_parser::node_iterator it = dimacs.nodes_begin();
            it != dimacs.nodes_end(); it++)
    { g.set_xy((*it).id_ - offset, (*it).x_, (*it).y_); }
    if(g.get_verbose()) { std::cerr << "nodes, converted" << std::endl; }

    // scan the list of edges so as to know the in and out degree of each node
    // this lets us allocate xy-graph memory contiguously
    std::vector<warthog::graph::ECAP_T> in_deg(dimacs.get_num_nodes(), 0);
    std::vector<warthog::graph::ECAP_T> out_deg(dimacs.get_num_nodes(), 0);
    for(warthog::dimacs_parser::edge_iterator it = dimacs.edges_begin();
            it != dimacs.edges_end(); it++)
    {
        in_deg.at(it->head_id_ - offset)++;
        out_deg.at(it->tail_id_ - offset)++;
    }

    // allocate memory for edges
    for(warthog::dimacs_parser::node_iterator it = dimacs.nodes_begin();
            it != dimacs.nodes_end(); it++)
    {
        uint32_t nid = (*it).id_ - offset;
        g.get_node(nid)->capacity(
            store_incoming_edges ? in_deg.at(nid) : 0, out_deg[nid]);
        g.set_xy(nid, (*it).x_, (*it).y_);
    }

    // convert edges to graph format
    for(warthog::dimacs_parser::edge_iterator it = dimacs.edges_begin();
            it != dimacs.edges_end(); it++)
    {
        uint32_t hid = (*it).head_id_ - offset;
        uint32_t tid = (*it).tail_id_ - offset;

        if(reverse_arcs)
        {
            uint32_t tmp = hid;
            hid = tid;
            tid = tmp;
        }
        warthog::graph::edge e;
        e.node_id_ = hid;
        e.wt_ = (*it).weight_;

#ifndef NDEBUG
        warthog::graph::ECAP_T deg_before = g.get_node(tid)->out_degree();
#endif

        g.get_node(tid)->add_outgoing(e);

#ifndef NDEBUG
        warthog::graph::ECAP_T deg_after = g.get_node(tid)->out_degree();
#endif

        // edges can be stored twice: once as an incoming edge
        // and once as an outgoing edge
        if(store_incoming_edges)
        {
            e.node_id_ = tid;
            g.get_node(hid)->add_incoming(e);

#ifndef NDEBUG
            // sanity check: either the edge was a duplicate (and nothing
            // was added) or the added was added in which case it should
            // be the last element in the outgoing list of the tail node
            warthog::graph::edge sanity =
                *(g.get_node(tid)->outgoing_end()-1);
            assert(deg_before == deg_after ||
                    sanity.node_id_ == hid);
#endif
        }
        if(g.get_verbose() && ((it - dimacs.edges_begin()) % 1000) == 0)
        {
            std::cerr
                << "\rconverted K edges "
                << (it - dimacs.edges_begin()) / 1000;
        }
    }
    g.is_euclidean(enforce_euclidean);
    if(g.get_verbose()) { std::cout << "edges, converted" << std::endl; }

    std::cerr << "edge memory fragmentation: (1=none): "
        << g.edge_mem_frag() << std::endl;
}

void
warthog::graph::parse_xy(
    std::istream &in,
    uint32_t &num_nodes,
    uint32_t &num_edges,
    std::vector<std::pair<uint32_t, warthog::graph::edge>> &edges,
    std::vector<std::pair<int32_t, int32_t>> &xy,
    std::vector<warthog::graph::ECAP_T> &in_degree,
    std::vector<warthog::graph::ECAP_T> &out_degree)
{
    while(in.good())
    {
        in >> std::ws;
        if(in.peek() == '#')
        {
            while(in.get() != '\n');
            continue;
        }

        if(in.peek() == 'n') { while(in.get() != ' '); } // "nodes" keyword
        in >> num_nodes;
        in >> std::ws;
        if(in.peek() == 'e') { while(in.get() != ' '); } // "edges" keyword
        in >> num_edges;
        in >> std::ws;
        break;
    }

    edges.resize(num_edges);
    xy.resize(num_nodes);
    in_degree.assign(num_nodes, 0);
    out_degree.assign(num_nodes, 0);

    uint32_t n_added = 0, e_added=0;
    while(in.good())
    {
        // read nodes data
        in >> std::ws;
        char c = in.get();

        if(c == 'v')
        {
            uint32_t id;
            int32_t x, y;
            in >> id >> x >> y;
            xy.at(id) = std::pair<int32_t, int32_t>(x, y);
            in >> std::ws; // trailing whitespace
            n_added++;
        }

        if(c == 'e')
        {
            uint32_t from_id, to_id;
            warthog::graph::edge_cost_t cost;

            in >> from_id >> to_id >> cost;
            edges.at(e_added) =
                std::pair<uint32_t, warthog::graph::edge>
                    (from_id, warthog::graph::edge(to_id, cost));

            assert(out_degree.at(from_id) != warthog::graph::ECAP_MAX);
            assert(out_degree.at(to_id) != warthog::graph::ECAP_MAX);
            out_degree.at(from_id)++;
            in_degree.at(to_id)++;
            e_added++;
        }
    }
    assert(n_added == num_nodes);
    assert(e_added == num_edges);
}
