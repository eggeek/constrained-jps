#include "search_node.h"

unsigned int warthog::search_node::refcount_ = 0;

std::ostream& operator<<(std::ostream& str, const warthog::search_node& sn)
{
    sn.print(str);

    return str;
}
