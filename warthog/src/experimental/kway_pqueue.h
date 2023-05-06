#ifndef WARTHOG_KWAY_PQUEUE_H
#define WARTHOG_KWAY_PQUEUE_H

// kway_pqueue.h
//
// An attempt at an priority queue with k-arity. Based on
// the binary heap implementation of warthog::kway_pqueue 
//
// @author: dharabor
// @created: 2018-05-05
//

#include "search_node.h"

#include <cassert>
#include <iostream>

namespace warthog
{

class search_node;

struct min_q 
{ static const bool is_min_ = true; };

struct max_q
{ static const bool is_min_ = false; };


template <class Comparator = warthog::cmp_less_search_node,
          class QType = warthog::min_q>
class kway_pqueue 
{
	public:
		kway_pqueue(unsigned int size)
            : maxsize_(size), minqueue_(QType().is_min_), queuesize_(0), 
            elts_(0)
        {
            resize(size);
        }

        ~kway_pqueue()
        {
            delete [] elts_;
        }

		// removes all elements from the kway_pqueue
        void
        clear()
        {
            queuesize_ = 0;
        }

		// reprioritise the specified element (up or down)
        void 
        decrease_key(warthog::search_node* val)
        {	
            assert(val->get_priority() < queuesize_);
            minqueue_ ?  
                heapify_up(val->get_priority()) :
                heapify_down(val->get_priority());
        }

        void 
        increase_key(warthog::search_node* val)
        {
            assert(val->get_priority() < queuesize_);
            minqueue_ ? 
                heapify_down(val->get_priority()) :
                heapify_up(val->get_priority());
        }

		// add a new element to the kway_pqueue
        void 
        push(warthog::search_node* val)
        {
            if(contains(val))
            {
                return;
            }

            if(queuesize_+1 > maxsize_)
            {
                resize(maxsize_*2);
            }
            unsigned int priority = queuesize_;
            elts_[priority] = val;
            val->set_priority(priority);
            queuesize_++;
            heapify_up(priority);
        }

		// remove the top element from the kway_pqueue
        warthog::search_node*
        pop()
        {
            if (queuesize_ == 0)
            {
                return 0;
            }

            warthog::search_node *ans = elts_[0];
            queuesize_--;

            if(queuesize_ > 0)
            {
                elts_[0] = elts_[queuesize_];
                elts_[0]->set_priority(0);
                heapify_down(0);
            }
            return ans;
        }

		// @return true if the priority of the element is 
		// otherwise
		inline bool
		contains(warthog::search_node* n)
		{
			unsigned int priority = n->get_priority();
			if(priority < queuesize_ && &*n == &*elts_[priority])
			{
				return true;
			}
			return false;
		}

		// retrieve the top element without removing it
		inline warthog::search_node*
		peek()
		{
			if(queuesize_ > 0)
			{
				return this->elts_[0];
			}
			return 0;
		}

		inline unsigned int
		size()
		{
			return queuesize_;
		}

		inline bool
		is_minqueue() 
		{ 
			return minqueue_; 
		} 
		
        void 
        print(std::ostream& out)
        {
            for(unsigned int i=0; i < queuesize_; i++)
            {
                elts_[i]->print(out);
                out << std::endl;
            }
        }

		unsigned int
		mem()
		{
			return maxsize_*sizeof(warthog::search_node*)
				+ sizeof(*this);
		}

	private:
		unsigned int maxsize_;
		bool minqueue_;
		unsigned int queuesize_;
		warthog::search_node** elts_;
        Comparator cmp_;
        uint32_t arity_ = 4;
        uint32_t log_arity_ = 2;

		// reorders the subkway_pqueue containing elts_[index]
        void 
        heapify_up(unsigned int index)
        {
            assert(index < queuesize_);
            while(index > 0)
            {
                unsigned int parent = (index-1) >> log_arity_;
                if(cmp_(*elts_[index], *elts_[parent]))
                {
                    swap(parent, index);
                    index = parent;
                }
                else { break; }
            }
        }
		
		// reorders the subkway_pqueue under elts_[index]
        void 
        heapify_down(unsigned int index)
        {
            unsigned int first_leaf_index = queuesize_ >> log_arity_;
            while(index < first_leaf_index)
            {
                // find smallest (or largest, depending on heap type) child
                uint32_t max_c = (queuesize_ - (index<<log_arity_));
                max_c = index + (max_c > arity_ ? arity_ : max_c);
                uint32_t best_c = index+1;
                for(uint32_t next_c = index+2; next_c <= max_c; next_c++)
                {
                    best_c = cmp_(*elts_[best_c], *elts_[next_c]) ?
                                    best_c : next_c;
                }

                //unsigned int best_c = (index<<1)+1;
                //unsigned int next_c = (index<<1)+2;
                //if((next_c < queuesize_) && *elts_[next_c] < *elts_[best_c])
                //{
                //    best_c = next_c;
                //}

                // swap child with parent if necessary
                if(cmp_(*elts_[best_c], *elts_[index]))
                {
                    swap(index, best_c);
                    index = best_c;
                }
                else { break; }
            }
        }

		// allocates more memory so the kway_pqueue can grow
        void
        resize(unsigned int newsize)
        {
            if(newsize < queuesize_)
            {
                std::cerr 
                    << "err; kway_pqueue::resize newsize < queuesize " 
                    << std::endl;
                exit(1);
            }

            warthog::search_node** tmp = new search_node*[newsize];
            for(unsigned int i=0; i < queuesize_; i++)
            {
                tmp[i] = elts_[i];
            }
            delete [] elts_;
            elts_ = tmp;
            maxsize_ = newsize;
        }

		// swap the positions of two nodes in the underlying array
		inline void 
		swap(unsigned int index1, unsigned int index2)
		{
			assert(index1 < queuesize_ && index2 < queuesize_);

			warthog::search_node* tmp = elts_[index1];
			elts_[index1] = elts_[index2];
			elts_[index1]->set_priority(index1);
			elts_[index2] = tmp;
			tmp->set_priority(index2);
		}
};

typedef kway_pqueue<warthog::cmp_less_search_node, warthog::min_q> kway_pqueue_min;
typedef kway_pqueue<warthog::cmp_greater_search_node, warthog::max_q> kway_pqueue_max;

}

#endif

