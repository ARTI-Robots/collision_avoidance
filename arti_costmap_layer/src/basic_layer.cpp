#include <arti_costmap_layer/basic_layer.h>

namespace arti_costmap_layer
{
BasicLayer::BasicLayer()
  : has_transformation_error_(false)
{
}

bool BasicLayer::hasTransformationError()
{
  return has_transformation_error_;
}
}
