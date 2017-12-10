#ifndef _GPS_TRANSFORMER_TASK_HPP_
#define _GPS_TRANSFORMER_TASK_HPP_

#include "gps_transformer/TaskBase.hpp"

namespace gps_transformer {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    explicit Task(std::string const& name = "gps_transformer::Task");
};

}  // namespace gps_transformer

#endif   // _GPS_TRANSFORMER_TASK_HPP_

