#ifndef __NOOP_H_
#define __NOOP_H_

// Search class that does nothing.
//
#include "problem_instance.h"
#include "search.h"
#include "solution.h"

namespace warthog
{

class noop_search : public search
{
    public:
        noop_search() { }
        virtual ~noop_search() { }

        virtual void
        get_path(warthog::problem_instance& pi, warthog::solution& sol)
        {
            sol.reset();
        }

        virtual void
        get_pathcost(warthog::problem_instance& pi, warthog::solution& sol)
        {
            sol.reset();
        }

        virtual size_t
        mem()
        {
            return sizeof(*this);
        }
};

}

#endif // __NOOP_H_
