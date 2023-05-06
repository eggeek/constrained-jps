#ifndef WARTHOG_GRAPH_H
#define WARTHOG_GRAPH_H

// graph.h
//
// Common data structures for graphs.
//
// @author: dharabor
// @created: 2016-05-02
//

#include <cassert>
#include <cstdint>
#include <cfloat>
#include <iostream>

#include "constants.h"

namespace warthog
{
namespace graph
{

// adjustable limit on the maximum edge capacity (== degree) of a node. 
// smaller values make node manipulation operations more cache friendly
typedef uint16_t ECAP_T;
const uint16_t ECAP_MAX = UINT16_MAX;

typedef double edge_cost_t;
const double EDGE_COST_MAX = DBL_MAX;

template<typename LABEL_T>
class edge_base
{
    public:
        edge_base() { node_id_ = UINT32_MAX; wt_ = UINT32_MAX; label_ = 0;}

        edge_base(uint32_t node_id, edge_cost_t wt, LABEL_T label)
        {
            node_id_ = node_id;
            wt_ = wt;
            label_ = label;
        }

        edge_base(uint32_t node_id, edge_cost_t wt)
        {
            node_id_ = node_id;
            wt_ = wt;
            label_ = 0;
        }

        edge_base(const warthog::graph::edge_base<LABEL_T>& other) 
        { node_id_ = other.node_id_; wt_ = other.wt_; label_ = other.label_; }

        edge_base&
        operator=(const warthog::graph::edge_base<LABEL_T>& other) 
        { 
            node_id_ = other.node_id_; wt_ = other.wt_; label_ = other.label_;
            return *this;
        }


        void
        print(std::ostream& out)
        {
            out << " edge endpoint " << node_id_ << "; cost " << wt_;
        }

        // we store only one id, typically the head node
        // the id of the other node is derived contextually
        uint32_t node_id_;
        edge_cost_t wt_;
        LABEL_T label_;
};
typedef edge_base<uintptr_t> edge;
typedef edge* edge_iter;

inline bool
operator==(const warthog::graph::edge e1, const warthog::graph::edge e2)
{
    uint32_t qword = 0;
    for( ; qword < sizeof(e1) >> 3; qword++)
    {
        if(((uint64_t*)&e1)[qword] != ((uint64_t*)&e2)[qword]) 
        { return false; }
    }

    uint64_t byte = qword * 8;
    for( ; byte < sizeof(e1); byte++)
    {
        if(((char*)&e1)[byte] != ((char*)&e2)[byte]) { return false; }
    }
    return true;
}

class node
{
    public:
        node()
        {
           init(0, 0);
        };

        node(const node& other)
        {
            init(0, 0);
            *this = other;
        }

        node(node&& other)
        {
            incoming_ = other.incoming_;
            in_cap_ = other.in_cap_;
            in_deg_ = other.in_deg_;

            outgoing_ = other.outgoing_;
            out_cap_ = other.out_cap_;
            out_deg_ = other.out_deg_;

            other.incoming_ = 0;
            other.outgoing_ = 0;
            other.out_cap_ = 0;
            other.out_deg_ = 0;
            other.in_cap_ = 0;
            other.in_deg_ = 0;
        }


        ~node()
        {
            delete [] incoming_;
            incoming_ = 0;
            in_cap_ = in_deg_ = 0;

            delete [] outgoing_;
            outgoing_ = 0;
            out_cap_ = out_deg_ = 0;
        }

        warthog::graph::node&
        operator=(warthog::graph::node&& other)
        {
            this->incoming_ = other.incoming_;
            this->in_deg_ = other.in_deg_;
            this->in_cap_ = other.in_cap_;

            this->outgoing_ = other.outgoing_;
            this->out_deg_ = other.out_deg_;
            this->out_cap_ = other.out_cap_;

            other.out_deg_ = 0;
            other.in_deg_ = 0;
            other.incoming_ = 0;
            other.outgoing_ = 0;

            return *this;
        }


        warthog::graph::node&
        operator=(const warthog::graph::node& other)
        {
            in_deg_ = other.in_deg_;
            in_cap_ = other.in_deg_;
            out_deg_ = other.out_deg_;
            out_cap_ = other.out_deg_;

            delete [] incoming_;
            delete [] outgoing_;
            incoming_ = new edge[other.in_cap_];
            outgoing_ = new edge[other.out_cap_];

            for(ECAP_T i = 0; i < other.in_deg_; i++)
            { incoming_[i] = other.incoming_[i]; }

            for(ECAP_T i = 0; i < other.out_deg_; i++)
            { outgoing_[i] = other.outgoing_[i]; }

            return *this;
        }

        inline warthog::graph::edge_iter
        add_incoming(warthog::graph::edge e)
        {
            return add_edge(e, in_cap_, in_deg_, incoming_);
        }

        inline warthog::graph::edge_iter
        add_outgoing(warthog::graph::edge e)
        {
            return add_edge(e, out_cap_, out_deg_, outgoing_);
        }

        // insert an outgoing edge at a fixed position in the 
        // underlying edges array. if the position is anywhere
        // except the end of the array the elements [index, end)
        // will be shifted to the right to make space. 
        // NB: this operation may involve a reallocation of storage
        inline warthog::graph::edge_iter
        insert_outgoing(warthog::graph::edge e, uint32_t index)
        {
            if(out_deg_ == out_cap_)
            {
                assert(out_cap_<<1 < ECAP_MAX);
                out_cap_ = increase_capacity((ECAP_T)(out_cap_*2), out_cap_, outgoing_);
            }
            assert(out_cap_ > 0 && out_cap_ > out_deg_);

            // shuffle everything down so we can insert the new edge
            for(uint32_t i = out_deg_; i > index; i--)
            {
                outgoing_[i] = outgoing_[i-1];
            }
            outgoing_[index] = e;
            out_deg_++;
            return &outgoing_[index];
        }

        inline void
        del_incoming(warthog::graph::edge_iter iter)
        {
            del_edge(iter, in_deg_, incoming_);
        }

        inline void
        del_outgoing(warthog::graph::edge_iter iter)
        {
            del_edge(iter, out_deg_, outgoing_);
        }

        inline edge_iter
        incoming_begin() const
        {
            //return incoming_;
            return (in_deg_ == 0 ? 0 : &incoming_[0]);
        }

        inline edge_iter
        incoming_end() const
        {
            //return &incoming_[in_deg_];
            return (in_deg_ == 0 ? 0 : &incoming_[in_deg_]);
        }

        inline edge_iter
        outgoing_begin() const
        {
            return (out_deg_ == 0 ? 0 : &outgoing_[0]);
            //return outgoing_;
        }

        inline edge_iter
        outgoing_end() const
        {
            return (out_deg_ == 0 ? 0 : &outgoing_[out_deg_]);
        }

        inline ECAP_T
        in_degree() const 
        {
            return in_deg_;
        }

        inline ECAP_T
        out_degree() const
        {
            return out_deg_;
        }

        // discard all incoming and all outgoing edges
        // after calling this function the in degree and out
        // degree of the node are both equal to zero
        inline void
        clear()
        {
            in_deg_ = 0;
            out_deg_ = 0;
        }

        // find the index of an outgoing edge whose head 
        // node is @param to_id. the search begins from 
        // the edge iterator specified in @param it
        // (by default, the first outgoing edge).
        // the function returns an iterator for the
        // target edge or ::outgoing_end() if no suh edge
        warthog::graph::edge_iter
        find_edge(uint32_t to_id, warthog::graph::edge_iter it = 0)
        {
            if(it == 0) { it = outgoing_begin(); }
            if(it < outgoing_begin() || it >= outgoing_end())
            {
                return outgoing_end();
            }

            for( ; it < outgoing_end(); it++)
            {
                if((*it).node_id_ == to_id)
                {
                    return it;
                }
            }
            return outgoing_end();
        }

        inline size_t
        in_capacity() { return in_cap_; }

        inline size_t
        out_capacity() { return out_cap_; }

        // resize the containers that store incoming and outgoing edges

        // EXPERIMENTAL; DO NOT USE
        inline void
        relocate(edge* tmp_in, edge* tmp_out)
        {
            for(uint32_t i = 0; i < in_deg_; i++)
            { tmp_in[i] = incoming_[i]; }
            for(uint32_t i = 0; i < out_deg_; i++)
            { tmp_out[i] = outgoing_[i]; }

            delete [] incoming_;
            delete [] outgoing_; 
            incoming_ = tmp_in;
            outgoing_ = tmp_out;
            out_cap_ = out_deg_;
            in_cap_ = in_deg_;
        }

        inline size_t
        mem()
        {
            return 
                sizeof(warthog::graph::edge) * in_cap_ + 
                sizeof(warthog::graph::edge) * out_cap_ +
                sizeof(this);
        }

        inline void
        capacity(warthog::graph::ECAP_T new_in_cap, 
                 warthog::graph::ECAP_T new_out_cap)
        {
            if(new_in_cap > in_cap_)
            {
                in_cap_ = increase_capacity(new_in_cap, in_cap_, incoming_);
            }
            if(new_out_cap > out_cap_)
            {
                out_cap_ = increase_capacity(new_out_cap, out_cap_, outgoing_);
            }
        }

    private:
        edge* incoming_;
        ECAP_T in_deg_;
        ECAP_T in_cap_;

        edge* outgoing_;
        ECAP_T out_deg_;
        ECAP_T out_cap_;

        void
        init(ECAP_T in_capacity, ECAP_T out_capacity)
        {
            outgoing_ = 0;
            out_deg_ = out_cap_ = 0;
            incoming_ = 0;
            in_deg_ = in_cap_ = 0;
            capacity(in_capacity, out_capacity);
        }

        // increase max (incoming or outgoing) edges that can be 
        // stored along with this node
        ECAP_T
        increase_capacity(ECAP_T newcap, ECAP_T oldcap, edge*& collection)
        {
            newcap = std::max<ECAP_T>(1, newcap);
            if(newcap <= oldcap) { return oldcap; }

            edge* newcollection = new edge[newcap];
            for(int i = 0; i < oldcap; i++)
            {
                newcollection[i] = collection[i];
            }
            delete [] collection;
            collection = newcollection;
            return newcap;
        }

        // remove an edge and shift the remaining edges
        // to plug the hole 
        // NB: doesn't free any memory!!
        inline void
        del_edge(warthog::graph::edge_iter elt, ECAP_T& deg, edge*& elts)
        {
            size_t index = (size_t)(&*elt - elts);
            if(index < deg)
            {
                deg--;
                for( ; index < deg; index++)
                {
                    elts[index] = elts[index+1];
                }
            }
        }

        // add an edge to one of the collections (in or out) of a node
        // NB: copy semantics
        inline warthog::graph::edge_iter
        add_edge(warthog::graph::edge& e, 
                ECAP_T& max_elts, ECAP_T& deg, edge*& elts)
        {
            if(deg == ECAP_MAX)
            {
                std::cerr << "warthog::graph::node edge-capacity reached!\n";
                return &elts[deg];
            }
         
            // don't add redundant edges; we only want one:
            // the one with lowest cost
            for(uint32_t i = 0; i < deg; i++)
            {
                if(elts[i].node_id_ == e.node_id_)
                {
                    if(e.wt_ < elts[i].wt_)
                    {
                        elts[i].wt_ = e.wt_;
                    }
                    return &elts[deg];
                }
            }

            if(deg == max_elts)
            {
                //uint32_t bigmax = (max_elts == 0 ? 1 : (2*max_elts));
                max_elts = increase_capacity((ECAP_T)(2*max_elts), max_elts, elts);
            }
            elts[deg] = e; 
            return &elts[deg++];
        }

};
typedef node* node_iter;

inline bool
operator==(const warthog::graph::node& n1, const warthog::graph::node& n2)
{
    if(!(n1.in_degree() == n2.in_degree()) &&
        (n1.out_degree() == n2.out_degree())) { return false; }
    
    for(uint32_t i = 0; i < n1.in_degree(); i++)
    {
        if(!(n1.incoming_begin()[i] == n2.incoming_begin()[i])) 
        { return false; }
    }

    for(uint32_t i = 0; i < n1.out_degree(); i++)
    {
        if(!(n1.outgoing_begin()[i] == n2.outgoing_begin()[i])) 
        { return false; }
    }
    return true;
}
        
// grid costs have double precision edge weights 
// but our graphs use integer costs; we scale
// up all the grid costs to equivalent integers.
// we also scale the x and y coordinates in the 
// same fashion.
const uint32_t GRID_TO_GRAPH_SCALE_FACTOR = warthog::ONE;

}
}

#endif
