#ifndef WARTHOG_ZERO_HEURISTIC_H
#define WARTHOG_ZERO_HEURISTIC_H

// zero_heuristic.h
//
// @author: dharabor
// @created: 2014-10-22
//

#include "constants.h"
#include "helpers.h"

#include <cstdlib>

namespace warthog
{

class zero_heuristic
{
	public:
		zero_heuristic() {}
		~zero_heuristic() {}

		inline double
		h(unsigned int x, unsigned int y, unsigned int x2, unsigned int y2)
		{
            return 0;
		}

		inline double
		h(warthog::sn_id_t id, warthog::sn_id_t id2)
		{
            return 0;
		}

        size_t
        mem() { return sizeof(this); } 
};

}

#endif

