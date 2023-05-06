#ifndef WARTHOG_HEAP_H
#define WARTHOG_HEAP_H

// heap.h
//
// A generic, updateable min/max priority queue. 
//
// @author: dharabor
// @created: 2016-05-18
//

#include "constants.h"

#include <cassert>
#include <functional>
#include <iostream>

namespace warthog
{

//template<typename T1, typename T2>
//class heap;

template<class T>
struct heap_node
{
    public:
        heap_node() : heap_index_(warthog::INF32) { } 
        heap_node(T elt) : elt_(elt), heap_index_(warthog::INF32) { }
        heap_node(const heap_node& other) 
        {
            elt_ = other.elt_;
            heap_index_ = other.heap_index_;
        }

        inline T& 
        get_element() { return elt_; }

        inline uint32_t 
        get_index() { return heap_index_; }

        // don't mess with this
        void
        set_index(uint32_t index) { heap_index_ = index; }
       
    private:    
        T elt_;

        // tracking the position of elements in the heap 
        // array allows us to quickly locate and reposition
        // elements in case their priority needs updating
        uint32_t heap_index_;
};

template<typename T, typename Comparator = std::less<T>>
class heap 
{
    typedef heap_node<T> heap_item;

	public:
		heap(unsigned int size, bool minqueue)
            : maxsize_(size), minqueue_(minqueue), 
            queuesize_(0), elts_(0)
        {
            resize(size);
        }

		~heap()
        {
            delete [] elts_;
        }

		// removes all elements from the heap
		void 
		clear()
        {
            queuesize_ = 0;
        }


		// reprioritise the specified element (up or down)
		void
		decrease_key(heap_item* val)
        {
            assert(val->get_index() < queuesize_);
            if(minqueue_)
            {
                heapify_up(val->get_index());
            }
            else
            {
                heapify_down(val->get_index());
            }
        }

		void 
		increase_key(heap_item* val)
        {
            assert(val->get_index() < queuesize_);
            if(minqueue_)
            {
                heapify_down(val->get_index());
            }
            else
            {
                heapify_up(val->get_index());
            }
        }

		// add a new element to the heap
		void 
		push(heap_item* val)
        {
            if(contains(val))
            {
                return;
            }

            if(queuesize_+1 > maxsize_)
            {
                resize(maxsize_*2);
            }
            unsigned int index = queuesize_;
            elts_[index] = val;
            val->set_index(index);
            queuesize_++;
            heapify_up(index);
        }

		// remove the top element from the heap
		heap_item*
		pop()
        {
            if (queuesize_ == 0)
            {
                return 0;
            }

            heap_item *ans = elts_[0];
            queuesize_--;

            if(queuesize_ > 0)
            {
                elts_[0] = elts_[queuesize_];
                elts_[0]->set_index(0);
                heapify_down(0);
            }
            return ans;
        }

		// @return true if the specified element is in 
        // the list. return false if it is not.
		inline bool
		contains(heap_item* n)
		{
			unsigned int index = n->get_index();
			if(index < queuesize_ && &*n == &*elts_[index])
			{
				return true;
			}
			return false;
		}

		// retrieve the top element without removing it
		inline heap_item*
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
			return maxsize_*sizeof(heap_item*)
				+ sizeof(*this);
		}

	private:
		unsigned int maxsize_;
		bool minqueue_;
		unsigned int queuesize_;
		heap_item** elts_;
        Comparator cmp_;

		// reorders the subheap containing elts_[index]
		void 
		heapify_up(unsigned int index)
        {
            assert(index < queuesize_);
            while(index > 0)
            {
                unsigned int parent = (index-1) >> 1;
                if(rotate(elts_[parent], elts_[index]))
                {
                    swap(parent, index);
                    index = parent;
                }
                else
                {
                    break;
                }
            }
        }
		
		// reorders the subheap under elts_[index]
		void 
		heapify_down(unsigned int index)
        {
            unsigned int first_leaf_index = queuesize_ >> 1;
            while(index < first_leaf_index)
            {
                // find smallest (or largest, depending on heap type) child
                unsigned int child1 = (index<<1)+1;
                unsigned int child2 = (index<<1)+2;
                unsigned int which = child1;
                
                if(child2 < queuesize_ &&
                    cmp_(elts_[child2]->get_element(),
                    elts_[child1]->get_element()))
                {
                    which = child2;
                }

                // swap child with parent if necessary
                if(cmp_(elts_[which]->get_element(), 
                        elts_[index]->get_element()))
                {
                    swap(index, which);
                    index = which;
                }
                else
                {
                    break;
                }
            }
        }

		// allocates more memory so the heap can grow
		void 
		resize(unsigned int newsize)
        {
        //	std::cout << "heap::resize oldsize: "<<queuesize_<<" newsize " << newsize<<std::endl;
            if(newsize < queuesize_)
            {
                std::cerr << "err; heap::resize newsize < queuesize " << std::endl;
                exit(1);
            }

            heap_item** tmp = new heap_item*[newsize];
            for(unsigned int i=0; i < queuesize_; i++)
            {
                tmp[i] = elts_[i];
            }
            delete [] elts_;
            elts_ = tmp;
            maxsize_ = newsize;
        }
	
		// returns true if:
		//   minqueue is true and the priority of second < first
		//   minqueue is false and the priority of second > first
		inline bool 
		rotate(heap_item* first, heap_item* second)
		{
			if(minqueue_)
			{
				if(cmp_(second->get_element(), first->get_element()))
				{
					return true;
				}
			}
			else
			{
				if(cmp_(first->get_element(), second->get_element()))
				{
					return true;
				}
			}
			return false;
		}

		// swap the positions of two nodes in the underlying array
		inline void 
		swap(unsigned int index1, unsigned int index2)
		{
			assert(index1 < queuesize_ && index2 < queuesize_);

			heap_item* tmp = elts_[index1];
			elts_[index1] = elts_[index2];
			elts_[index1]->set_index(index1);
			elts_[index2] = tmp;
			tmp->set_index(index2);
		}
};

}

#endif

