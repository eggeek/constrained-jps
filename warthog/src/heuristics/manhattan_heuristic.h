#ifndef WARTHOG_MANHATTAN_HEURISTIC_H
#define WARTHOG_MANHATTAN_HEURISTIC_H

// manhattan_heuristic.h
//
// @author: dharabor
// @created: 21/08/2012
//

#include "constants.h"
#include "helpers.h"

#include <cstdlib>

namespace warthog
{

class manhattan_heuristic
{
	public:
		manhattan_heuristic(uint32_t mapwidth, uint32_t mapheight)
		 : mapwidth_(mapwidth) 
        { }

		~manhattan_heuristic() {}

		inline double
		h(int32_t x, int32_t y, int32_t x2, int32_t y2)
		{
            // NB: precision loss when double is an integer
			return (abs(x-x2) + abs(y-y2));
		}

		inline double
		h(warthog::sn_id_t id, warthog::sn_id_t id2)
		{
			int32_t x, x2;
			int32_t y, y2;
			warthog::helpers::index_to_xy((uint32_t)id, mapwidth_, x, y);
			warthog::helpers::index_to_xy((uint32_t)id2, mapwidth_, x2, y2);
			return this->h(x, y, x2, y2);
		}

        size_t
        mem() { return sizeof(this); }

	private:
		uint32_t mapwidth_;
};

}

#endif

