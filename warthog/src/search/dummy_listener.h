#ifndef WARTHOG_DUMMY_LISTENER_H
#define WARTHOG_DUMMY_LISTENER_H

// search/dummy_listener.h
//
// A search listener is a callback class that executes specialised
// code for partiular search events, such as when:
//  - a node is generated
//  - a node is expanded
//  - a node is relaxed
//
//  This class implements dummy listener with empty event handlers.
//
// @author: dharabor
// @created: 2020-03-09
//

#include "sys/constants.h"
#include "sys/forward.h"

namespace warthog
{

class dummy_listener
{
    public:

        inline void
        generate_node(warthog::search_node* parent, 
                      warthog::search_node* child, 
                      warthog::cost_t edge_cost,
                      uint32_t edge_id) { } 

        inline void
        expand_node(warthog::search_node* current) { }

        inline void
        relax_node(warthog::search_node* current) { }

};

}

#endif
