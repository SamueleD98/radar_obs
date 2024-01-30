
#ifndef RADAR_OBS_UTILS_HPP
#define RADAR_OBS_UTILS_HPP

#include "radar_obs/publisher.hpp"

// USING NED

Eigen::Vector2d GetLocal(ctb::LatLong LatLong){
  Eigen::Vector3d pos;
  ctb::LatLong2LocalNED(LatLong, 0, centroid, pos);
  return Eigen::Vector2d(pos.x(), pos.y());
}

ctb::LatLong GetLatLong(Eigen::Vector2d posLocal){
  ctb::LatLong LatLong;
  double alt;
  Eigen::Vector3d pos(posLocal.x(), posLocal.y(), 0);
  ctb::LocalNED2LatLong(pos, centroid, LatLong, alt);
  return LatLong;
}

#endif //RADAR_OBS_UTILS_HPP
