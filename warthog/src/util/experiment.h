#ifndef WARTHOG_EXPERIMENT_H
#define WARTHOG_EXPERIMENT_H

// experiment.h
//
// An object for holding experiments read from Nathan Sturtevant's 
// .scenario files.
// Based on an implementation from HOG by Renee Jansen.
//
// NB: This implementation makes use of an additional attribute,
// ::precision_, which can be used to indicate the accuracy with which
// ::distance_ should be interpreted. The hardcoded default is 4.
//
// NB2: The attributes ::mapwidth_ and ::mapheight_ refer to the x/y dimensions
// that ::map should be scaled to. The individial node
// coordinates (::startx_, ::starty_ etc.) are taken with respect to the 
// dimensions of the scaled map.
//
// @author: dharabor
// @created: 21/08/2012
//

#include "problem_instance.h"

#include <iostream>
#include <string>

namespace warthog
{

class experiment
{
	public:
		experiment(uint32_t sx, uint32_t sy, uint32_t gx, 
				uint32_t gy, uint32_t mapwidth, uint32_t mapheight,
			   	double d, std::string m) :
			startx_(sx), starty_(sy), goalx_(gx), goaly_(gy), 
			mapwidth_(mapwidth), mapheight_(mapheight), distance_(d), map_(m),
			precision_(4)
		{}
		~experiment() {}

		inline uint32_t
		startx() { return startx_; }

		inline uint32_t
		starty() { return starty_; }

		inline uint32_t
		goalx() { return goalx_; } 

		inline uint32_t
		goaly()  { return goaly_; }

		inline double
		distance() { return distance_; }

		inline std::string
		map() { return map_; } 

		inline uint32_t
		mapwidth() { return mapwidth_; }

		inline uint32_t
		mapheight() { return mapheight_; }

		inline int32_t
		precision() { return precision_; }

		inline void
		set_precision(int32_t prec) { precision_ = prec; }

		void
		print(std::ostream& out);

        warthog::problem_instance
        get_instance()
        {
            return warthog::problem_instance(
                    starty_ * mapwidth_ + startx_, 
                    goaly_ * mapwidth_ + goalx_);
        }

	private:
		uint32_t startx_, starty_, goalx_, goaly_;
		uint32_t mapwidth_, mapheight_;
		double distance_;
		std::string map_;
		int32_t precision_;

		// no copy
		experiment(const experiment& other) {} 
		experiment& operator=(const experiment& other) { return *this; }
};

}

#endif

