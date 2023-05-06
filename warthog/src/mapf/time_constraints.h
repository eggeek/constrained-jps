#ifndef WARTHOG_MAPF_TIME_CONSTRAINTS
#define WARTHOG_MAPF_TIME_CONSTRAINTS

// mapf/time_constraints.h
//
// This data structure describes the all constraints that currently apply
// to an associated grid map. A constraint describes: 
//  (i) the cost of reaching the cell (i.e. a cost on the vertex)
//  (ii) a set of edge labels that describe the costs of move actions 
//  applicable  in the cell (once reached) 
//  (iii) the timestep when the constraint applies
// 
// @author: dharabor
// @created: 2018-12-06
//

namespace warthog
{
namespace mapf
{

// The constraint data type.
//
// v_: indicates if the tile is blocked or not
// the default state is unblocked
//
// e_: indicates the cost of each move action.
// the default cost for each action is 1. 
// the special value warthog::INF indicates the move cannot be executed.
// 
// timestep_: indicates the timestep when the edge costs apply. 
// the default timestep is 0.
// 
struct cell_constraint
{
    cell_constraint()
    {
        v_ = 0;
        e_[warthog::cbs::move::NORTH] = 1;
        e_[warthog::cbs::move::SOUTH] = 1;
        e_[warthog::cbs::move::EAST] = 1;
        e_[warthog::cbs::move::WEST] = 1;
        e_[warthog::cbs::move::WAIT] = 1;
        timestep_ = 0;
    }

    cell_constraint&
    operator=(const cell_constraint& other)
    {
        v_ = other.v_;
        e_[0] = other.e_[0];
        e_[1] = other.e_[1];
        e_[2] = other.e_[2];
        e_[3] = other.e_[3];
        e_[4] = other.e_[4];
        timestep_ = other.timestep_;
        return *this;
    }

    bool v_;
    double e_[5];
    uint16_t timestep_;
};

template<typename CONSTRAINT>
class time_constraints
{
    public:

       time_constraints(uint32_t map_xy_sz) : map_xy_sz_(map_xy_sz)
       {
           cons_ = new std::vector< std::vector<CONSTRAINT> >(map_xy_sz_);
       } 

       ~time_constraints()
       {
           delete cons_;
       }

       // add a constraint to location @param xy_id
       inline void
       add_constraint(uint32_t xy_id, CONSTRAINT& con)
       {
           assert(xy_id < cons_->size());
           CONSTRAINT* cur_con = 
               get_constraint(xy_id, con.timestep_);
           if(!cur_con)
           {
               cons_->at(xy_id).push_back(con);
           }
           else
           {
                *cur_con = con;
           }
       }

       // return all constraints associated with the xy location
       // @param padded_id
       inline std::vector<CONSTRAINT>& 
       get_constraint_set(uint32_t xy_id)
       {
           return cons_->at(xy_id);
       }

       // return the constraint associated with the location 
       // @param xy_id, at the time @param timestep
       inline CONSTRAINT*
       get_constraint(uint32_t xy_id, uint32_t timestep)
       {
            auto retval = 
                std::find_if(
                        (*cons_)[xy_id].begin(),
                        (*cons_)[xy_id].end(),
                    [timestep](CONSTRAINT& tmp)
                    -> bool
                    {
                        return tmp.timestep_  == timestep;
                    });
            if(retval == (*cons_)[xy_id].end())
            {
                return 0;
            }
            return &*retval;
       }

       // create or return the constraint associated with the location
       // @param xy_id, at the time @param timestep
       inline CONSTRAINT*
       get_or_create_constraint(uint32_t xy_id, uint32_t timestep)
       {
            auto retval =
                std::find_if(
                        (*cons_)[xy_id].begin(),
                        (*cons_)[xy_id].end(),
                    [timestep](CONSTRAINT& tmp)
                    -> bool
                    {
                        return tmp.timestep_  == timestep;
                    });
            if(retval == (*cons_)[xy_id].end())
            {
                (*cons_)[xy_id].emplace_back();
                (*cons_)[xy_id].back().timestep_ = (uint16_t)timestep;
                return &(*cons_)[xy_id].back();
            }
            return &*retval;
       }

       // remove all constraints associated with location
       void
       clear_constraint_set(uint32_t xy_id)
       {
           cons_->at(xy_id).clear();
       }

       // remove all constraints
       void
       clear()
       {
           for(uint32_t i = 0; i < cons_->size(); i++)
           {
               cons_->at(i).clear();
           }   
       }

    private:
        std::vector< std::vector< CONSTRAINT > >* cons_;
        CONSTRAINT dummy_;
        uint32_t map_xy_sz_;

}; // time_constraints


} // mapf
} // warthog

#endif

