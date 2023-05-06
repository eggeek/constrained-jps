#include "mapf/plan.h"


// load an existing plan
std::istream&
warthog::mapf::operator>>(std::istream& in, warthog::mapf::plan& theplan)
{
    char buf[1024];

    // skip comment lines

    // new path
    while(in.good())
    {
        if(in.peek() == '\n' || in.peek() == ' ' || in.peek() == '\t')
        {
            in.get(); 
            continue; 
        }

        if(in.peek() == '#') 
        {
            in.getline(buf, 1024);
            continue;
        }

        if(in.peek() == 'p')
        {
            theplan.paths_.push_back(warthog::solution());    
            in.getline(buf, 1024);
            continue;
        }

        if(in.eof()) { break; }

        // NB: each location is a time indexed location
        // the time is in the upper 4 bytes, the location
        // is in the lower 4 bytes.
        warthog::sn_id_t node_id;
        in >> node_id;
        theplan.paths_.back().path_.push_back(node_id);
    }
    return in;
}

// write out the current plan
std::ostream&
warthog::mapf::operator<<(std::ostream& out, warthog::mapf::plan& theplan)
{
    for(uint32_t i = 0; i < theplan.paths_.size(); i++)
    {
        out << "p " << i << " " << std::endl;
        for(uint32_t j = 0; j < theplan.paths_.at(i).path_.size(); j++)
        {
            out << theplan.paths_.at(i).path_.at(j) << std::endl;
        }
    }
    return out;
}
