#ifndef WARTHOG_EXPANSION_POLICY_H
#define WARTHOG_EXPANSION_POLICY_H

// expansion_policy.h
//
// An (abstract) expansion policy for searching explicit
// domains. It is assumed the number of nodes in the 
// search space is known apriori and a description of each node can be
// generated in constant time and independent of any other node.
//
// @author: dharabor
// @created: 2016-01-26
//

#include "arraylist.h"
#include "node_pool.h"
#include "search_node.h"
#include "problem_instance.h"

#include <vector>

namespace warthog
{

class expansion_policy
{
    public:
        expansion_policy(size_t nodes_pool_size);
        virtual ~expansion_policy();

        size_t
        get_nodes_pool_size() { return nodes_pool_size_; } 
       
        inline void
        reclaim()
        {
            //reset();
            //nodepool_->eclaim();
        }        

        inline mem::node_pool* get_nodepool() {
          return nodepool_;
        }

		inline void
		reset()
		{
			current_ = 0;
            neis_->clear();
		}

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            current_ = 0;
            n(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(current_ < neis_->size())
            {
                ret = (*neis_)[current_].node_;
                cost = (*neis_)[current_].cost_;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

        // return the nth successor 
        // NB: also adjust the current neighbour index such that the 
        // subsequent call to ::next will return the nth+1 neighbour.
        inline void
        get_successor(uint32_t which, warthog::search_node*& ret, double& cost)
        {
            if(which < neis_->size())
            {
                current_ = which;
                n(ret, cost);
            }
            else
            {
                ret = 0;
                cost = 0;
            }
        }

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            current_++;
            n(ret, cost);
		}

        inline size_t 
        num_successors() { return neis_->size(); } 

		virtual size_t
		mem()
		{
			return sizeof(*this) + 
            sizeof(neighbour_record) * neis_->capacity() + 
            nodepool_->mem();
		}

        // the expand function is responsible for generating the 
        // successors of a search node. each generated successor
        // should be added to the list of neighbours via 
        // ::add_neighbour
		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*) = 0;

        // this function creates a warthog::search_node object for
        // represent a given start state described by @param pi.
        // the simplest concrete implementation is to call ::generate but 
        // this assumes the identifier specified by @param pi is the same
        // as the one used internally by the domain model the expansion
        // policy is wrapping (e.g. a grid or a graph)
        //
        // @param pi: an problem describing a concrete start state
        // @return: a warthog::search_node object representing 
        // the start state. if the start state is invalid the
        // function returns 0
        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi) = 0;

        // this function creates a warthog::search_node object for
        // represent a given target state described by @param pi.
        // the simplest concrete implementation is to call ::generate but 
        // this assumes the identifier specified by @param pi is the same
        // as the one used internally by the domain model the expansion
        // policy is wrapping (e.g. a grid or a graph)
        //
        // @param pi: an problem describing a concrete target state
        // @return: a warthog::search_node object representing 
        // the target state. if the target state is invalid the
        // function returns 0
        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi) = 0;
      
        virtual void
        get_xy(sn_id_t node_id, int32_t& x, int32_t& y) = 0;

        // check if a given search node @param n corresponds to the
        // target. we do this here to decouple the internal 
        // representation of states from the search algorithm which
        // only knows about warthog::search_node objects.
        bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            return n->get_id() == pi->target_id_;
        }

        // get a search_node memory pointer associated with @param node_id. 
        // (value is null if @param node_id is bigger than nodes_pool_size_)
		inline warthog::search_node*
		generate(warthog::sn_id_t node_id)
		{
            return nodepool_->generate(node_id);
		}

        // get the search_node memory pointer associated with @param node_id
        // value is null if this node has not been previously allocated 
        // or if node_id is bigger than nodes_pool_size_
        warthog::search_node*
        get_ptr(sn_id_t node_id, uint32_t search_number)
        {
            warthog::search_node* tmp = nodepool_->get_ptr(node_id);
            if(tmp && tmp->get_search_number() == search_number) 
            {
                return tmp;
            }
            return 0;
        }

    protected:
        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
            //std::cout << " neis_.size() == " << neis_->size() << std::endl;
        }

        // return the index associated with the successor ::n()
        inline uint32_t 
        get_current_successor_index() { return current_; }

    private:

        struct neighbour_record
        {
            neighbour_record(warthog::search_node* node, double cost)
            {
                node_ = node;
                cost_ = cost;
            }
            warthog::search_node* node_;
            double cost_;
        };

        warthog::mem::node_pool* nodepool_;
        //std::vector<neighbour_record>* neis_;
        arraylist<neighbour_record>* neis_;
        uint32_t current_;
        size_t nodes_pool_size_;
};

}

#endif
