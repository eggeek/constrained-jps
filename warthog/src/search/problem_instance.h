#ifndef WARTHOG_PROBLEM_INSTANCE_H
#define WARTHOG_PROBLEM_INSTANCE_H

#include "search_node.h"

namespace warthog
{

class problem_instance
{
	public:
        problem_instance(
                warthog::sn_id_t startid, 
                warthog::sn_id_t targetid=warthog::SN_ID_MAX, 
                bool verbose=0) :
            start_id_(startid), 
            target_id_(targetid), 
            instance_id_(instance_counter_++),
            verbose_(verbose),
            extra_params_(0)

        { }

		problem_instance() :
            start_id_(warthog::SN_ID_MAX),
            target_id_(warthog::SN_ID_MAX),
            instance_id_(instance_counter_++),
            verbose_(0),
            extra_params_(0)
        { }


		problem_instance(const warthog::problem_instance& other)
        {
            this->start_id_ = other.start_id_;
            this->target_id_ = other.target_id_;
            //this->instance_id_ = other.instance_id_;
            this->instance_id_ = instance_counter_++;
            this->verbose_ = other.verbose_;
            this->extra_params_ = other.extra_params_;
        }

		~problem_instance() { }

        void
        reset()
        {
            instance_id_ = instance_counter_++;
        }

		warthog::problem_instance&
		operator=(const warthog::problem_instance& other)
        {
            this->start_id_ = other.start_id_;
            this->target_id_ = other.target_id_;
            //this->instance_id_ = other.instance_id_;
            this->instance_id_ = instance_counter_++;
            this->verbose_ = other.verbose_;
            this->extra_params_ = other.extra_params_;
            return *this;
        }

        void
        print(std::ostream& out)
        {
            out << "problem instance; start_id = " << start_id_ << " "
                << " target_id " << target_id_ << " " << " search_id "
                << instance_id_;
        }

		warthog::sn_id_t start_id_;
		warthog::sn_id_t target_id_;
		uint32_t instance_id_;
        bool verbose_;

        // stuff we might want to pass in
        void* extra_params_;

        private:
            static uint32_t instance_counter_;

};


}

std::ostream& operator<<(std::ostream& str, warthog::problem_instance& pi);

#endif
