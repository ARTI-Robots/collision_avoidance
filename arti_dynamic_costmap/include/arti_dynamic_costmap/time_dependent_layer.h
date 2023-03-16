#ifndef ARTI_DYNAMIC_COSTMAP_TIME_DEPENDEND_LAYER_H
#define ARTI_DYNAMIC_COSTMAP_TIME_DEPENDEND_LAYER_H

#include <ros/time.h>

#include <arti_costmap_layer/basic_layer.h>

namespace arti_dynamic_costmap
{
class TimeDependentLayer : public arti_costmap_layer::BasicLayer
{
public:
  virtual void useCostmapAt(const ros::Time& costmap_time) = 0;
};
}

#endif //ARTI_DYNAMIC_COSTMAP_TIME_DEPENDEND_LAYER_H
