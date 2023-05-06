#include "ch_data.h"

std::ofstream&
warthog::ch::operator<<(std::ofstream& fs_out, warthog::ch::ch_data& chd)
{
    // comments
    fs_out << "# contraction hierarchy data generated from\n"
        << "# " << chd.g_->get_filename() << std::endl
        << "# we use 32bit integer data formatted as follows: node data first, then edge data\n"
        << "# node data format: v [id] [x] [y] [level]\n"
        << "# edge data format: e [from_node_id] [to_node_id] [cost]" << std::endl;

    // header stuff
    //fs_out << "chd 1.0" << std::endl;
    fs_out
        << "nodes " << chd.g_->get_num_nodes() << " "
        << "edges " << chd.g_->get_num_edges_out() << std::endl;

    // node data
    for(uint32_t i = 0; i < chd.g_->get_num_nodes(); i++)
    {
        int32_t x, y;
        chd.g_->get_xy(i, x, y);
        fs_out
            << "v " << i << " "
            << x << " "
            << y << " ";

         uint32_t level;
         if(i >= chd.level_->size())
         { level = (uint32_t) chd.level_->size()-1; }
         else
         { level = chd.level_->at(i); }
         fs_out << level << std::endl;
    }

    // edge data. [===== NB ==== THIS NEXT BIT IS IMPORTANT ====]
    // contraction hierarchy data can be stored in two different
    // ways, depending on whether query algorithm is bidirectional
    // or forward search. For the former, the CH is stored as an
    // "up only" graph where all outgoing edges go up and where
    // all incoming edges are from higher level nodes.
    // For the latter, the CH is stored as an "up/down" graph
    // where all outgoing edges go up and all down edges are
    // stored in the incoming list
    for(uint32_t i = 0; i < chd.g_->get_num_nodes(); i++)
    {
        // write out all outgoing edges
        warthog::graph::node* n = chd.g_->get_node(i);
        uint32_t n_level = chd.level_->at(i);
        for(uint32_t idx = 0; idx < n->out_degree(); idx ++)
        {
            warthog::graph::edge* e = n->outgoing_begin()+idx;
            fs_out << "e " << i << " " << e->node_id_ << " " << e->wt_ << std::endl;
        }

        // write out any incoming up edges  (for UP_ONLY hierarchies)
        if(chd.type_ == warthog::ch::ch_type::UP_ONLY)
        {
            for(uint32_t edge_idx = 0; edge_idx < n->in_degree(); edge_idx++)
            {
                warthog::graph::edge* e = n->incoming_begin()+edge_idx;
                assert(n_level < chd.level_->at(e->node_id_));
                fs_out << "e " << e->node_id_ << " " << i  << " " << e->wt_ << std::endl;
            }
        }
    }
    return fs_out;
}

std::ifstream&
warthog::ch::operator>>(std::ifstream& fs_in, warthog::ch::ch_data& chd)
{
    uint32_t num_nodes=0, num_edges=0;
    while(fs_in.good())
    {
        fs_in >> std::ws;
        if(fs_in.peek() == '#')
        {
            while(fs_in.get() != '\n');
            continue;
        }

        if(fs_in.peek() == 'n') { while(fs_in.get() != ' '); } // "nodes" keyword
        fs_in >> num_nodes;
        fs_in >> std::ws;
        if(fs_in.peek() == 'e') { while(fs_in.get() != ' '); } // "edges" keyword
        fs_in >> num_edges;
        fs_in >> std::ws;
        break;
    }

    chd.g_->grow(num_nodes);
    chd.level_->resize(num_nodes);
    chd.up_degree_->resize(num_nodes, 0);

    uint32_t n_added = 0, e_added=0;
    while(fs_in.good())
    {
        // read nodes data
        fs_in >> std::ws;
        while(fs_in.peek() == 'v')
        {
            uint32_t id, level;
            int32_t x, y;
            fs_in.get(); // eat the 'v' char
            fs_in >> id >> x >> y >> level;
            chd.g_->set_xy(id, x, y);
            chd.level_->at(id) = level;
            fs_in >> std::ws; // trailing whitespace
            n_added++;
        }

        while(fs_in.peek() == 'e')
        {
            uint32_t from_id, to_id;
            warthog::graph::edge_cost_t cost;

            fs_in.get(); // eat the 'e' char
            fs_in >> from_id >> to_id >> cost;

            bool up_edges_finished = false;
            if(chd.level_->at(from_id) <= chd.level_->at(to_id))
            {
                // outgoing up edges are added to the outgoing list
                assert(up_edges_finished == false);
                warthog::graph::node* from = chd.g_->get_node(from_id);
                from->add_outgoing(warthog::graph::edge(to_id, cost));
                chd.up_degree_->at(from_id) = chd.up_degree_->at(from_id) + 1;
            }
            else
            {
                up_edges_finished = true;
                //switch(chd.type_)
                //{
                    // when the graph is "up only" (e.g. as per the algorithm BCH)
                    // we treat every outgoing down edge as incoming up edge
                    //case warthog::ch::UP_ONLY:
                    if(chd.type_ == warthog::ch::UP_ONLY)
                    {
                        warthog::graph::node* to = chd.g_->get_node(to_id);
                        to->add_incoming(warthog::graph::edge(from_id, cost));
                        break;
                    }

                    // when the graph is up/down (e.g. as per algorithm FCH)
                    // we add every outgoing down edge to the outgoing list
                    // but we always add to the front of the list
                    //case warthog::ch::UP_DOWN:
                    //default:
                    {
                        warthog::graph::node* from = chd.g_->get_node(from_id);
                        from->add_outgoing(warthog::graph::edge(to_id, cost));
                        break;
                    }
                //}
            }
            e_added++;
            fs_in >> std::ws;
        }
    }

    std::cerr << "ch graph, loaded.\n";
    std::cerr
        << "read " << n_added << " nodes (total " << num_nodes << ")"
        << " and read " << e_added << " edges (total "<< num_edges << ")"
        << std::endl;
    return fs_in;
}
