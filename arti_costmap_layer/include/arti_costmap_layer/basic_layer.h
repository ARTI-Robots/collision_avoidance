#ifndef ARTI_COSTMAP_LAYER_BASIC_LAYER_H
#define ARTI_COSTMAP_LAYER_BASIC_LAYER_H

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_layer.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_costmap_layer
{
class BasicLayer : public costmap_2d::CostmapLayer
{
public:
  BasicLayer();

  bool hasTransformationError();

protected:
  bool has_transformation_error_;
};
}


#endif //ARTI_COSTMAP_LAYER_BASIC_LAYER_H
