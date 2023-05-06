#ifndef WARTHOG_BINARY_H
#define WARTHOG_BINARY_H

// binary.h
//
// Binary search stuff
//
// @author: dharabor
// @created: 2017-01-10
//

#include <stdint.h>

namespace warthog
{

namespace util
{

// Binary search to find an element in the range [begin, end). The target
// element is specified indirectly by use of a functor, @param lessThan.
// The functor lessThan returns true if the sought element is < its first 
// argument, and false otherwise.
template<class t_iter, class t_functor>
t_iter
binary_find_first(t_iter begin, t_iter end, t_functor& lessThan)
{
    while(begin<(end-1))
    {
        uint32_t mid = begin + ((end-begin)>>1);
        if(lessThan(mid)) { end = mid ;  }
        else { begin = mid; }
    }
    return begin;
}

}

}

#endif
