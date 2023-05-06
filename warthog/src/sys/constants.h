#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

// constants.h
//
// @author: dharabor
// @created: 01/08/2012
//

#include <cfloat>
#include <cmath>
#include <climits>
#include <stdint.h>

namespace warthog
{
    typedef uint64_t sn_id_t; // address space for state identifiers
    static const sn_id_t SN_ID_MAX = UINT64_MAX;
    static const sn_id_t NO_PARENT = SN_ID_MAX;

	// each node in a weighted grid map uses sizeof(dbword) memory.
	// in a uniform-cost grid map each dbword is a contiguous set
	// of nodes s.t. every bit represents a node.
	typedef uint8_t dbword;

	// gridmap constants
	static const uint32_t DBWORD_BITS = sizeof(warthog::dbword)*8;
	static const uint32_t DBWORD_BITS_MASK = (warthog::DBWORD_BITS-1);
	static const uint32_t LOG2_DBWORD_BITS = static_cast<uint32_t>(ceil(log10(warthog::DBWORD_BITS) / log10(2)));

	// search and sort constants
	static const double DBL_ONE = 1.0f;
	static const double DBL_TWO = 2.0f;
	static const double DBL_ROOT_TWO = 1.414213562f;
	static const double DBL_ONE_OVER_TWO = 0.5;
	static const double DBL_ONE_OVER_ROOT_TWO = 1.0/DBL_ROOT_TWO;//0.707106781f;
	static const double DBL_ROOT_TWO_OVER_FOUR = DBL_ROOT_TWO*0.25;
  static const int32_t ONE = 100000;

  static const uint32_t INF32 = UINT32_MAX; // indicates uninitialised or undefined values 
  static const uint64_t INFTY = UINT64_MAX; // indicates uninitialised or undefined values 
  static const int MAXSIDE = INF32;

  typedef double cost_t;
  static const cost_t COST_MAX = DBL_MAX; 
  static const cost_t COST_MIN = DBL_MIN;

	// hashing constants
	static const uint32_t FNV32_offset_basis = 2166136261;
	static const uint32_t FNV32_prime = 16777619;

}

#endif


