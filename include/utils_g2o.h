#ifndef UTILS_G2O_H_
#define UTILS_G2O_H_

// g2o-dependent helpers extracted from utils.h so the network/feature path
// (super_point, super_glue, light_glue, plnet, ros_publisher) can be compiled
// without g2o on the include path. Files that work with line geometry pull in
// this header explicitly: mapline.cc, map.cc, line_processor.cc, the g2o
// optimization edges/vertices, and map_refiner.cc.

#include "utils.h"

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <boost/serialization/serialization.hpp>
#include <memory>

typedef std::shared_ptr<g2o::Line3D> Line3DPtr;
typedef std::shared_ptr<const g2o::Line3D> ConstLine3DPtr;

// boost serialization for Line3D (via Cartesian round-trip).
template <class Archive>
void SerializeLine3D(Archive &ar, Line3DPtr &line, const unsigned int version){
  g2o::Vector6 v;
  if (Archive::is_saving::value){
    v = line->toCartesian();
  }

  ar & boost::serialization::make_array(v.data(), v.size());

  if (Archive::is_loading::value){
    g2o::Line3D line_3d = g2o::Line3D::fromCartesian(v);
    line = std::make_shared<g2o::Line3D>(line_3d);
  }
}

#endif  // UTILS_G2O_H_
