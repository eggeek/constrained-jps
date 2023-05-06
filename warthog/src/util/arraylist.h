#ifndef WARTHOG_ARRAYLIST_H
#define WARTHOG_ARRAYLIST_H

// arraylist.h
//
// A dumb but dynamically-sized array for both simple and complex types.
// The array is initialised with an initial size. New objects are added 
// to the end of the collection. When the number of elements exceeds 
// the capacity of the array, its size increases by a factor of 2.
//
// @author: dharabor
// @created: 22/05/2013
//

#include <cstring>
#include <stdint.h>

namespace warthog
{
	template <class T>
	class arraylist
	{
		public:
			typedef T* iterator;

			arraylist(size_t size=100) : max_size_(size), terminator_(0)
			{
				collection_ = new uint8_t[max_size_*sizeof(T)];	
				next_ = 0;
			}

			~arraylist()
			{
				delete [] collection_;
			}

			inline T
			at(uint32_t index)
			{
				return ((T*)collection_)[index];
			}

			inline T
			operator[](uint32_t index)
			{
				return ((T*)collection_)[index];
			}

			inline void
			push_back(T element)
			{
				if(next_ < max_size_)
				{
					((T*)collection_)[next_] = element;	
					next_++;
				}
				else
				{
					grow(size()*2);
					((T*)collection_)[next_] = element;
					next_++;
				}
			}

			inline void
			pop_back()
			{
				if(next_ > 0)
				{
					next_--;
				}
			}

			inline void
			clear() 
			{
				next_ = 0;
			}

			inline size_t
			size()
			{
				return next_;
			}

            inline size_t
            capacity()
            {
                return max_size_;
            }

			size_t
			mem() { return sizeof(T)*max_size_;}


		private:
			uint8_t* collection_;
			size_t next_;
			size_t max_size_;
			const typename warthog::arraylist<T>::iterator terminator_;

			inline void 
			grow(size_t newsize)
			{
				if(newsize <= max_size_) { return; }

				uint8_t* bigcollection = new uint8_t[newsize*sizeof(T)];
				for(size_t i = 0; i < max_size_; i++)
				{
					((T*)bigcollection)[i] = ((T*)collection_)[i];
				}
				delete [] collection_;
				collection_ = bigcollection;
				max_size_ = newsize;
			}

	};
}

#endif

