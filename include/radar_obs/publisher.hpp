#ifndef RADAR_OBS_PUBLISHER_HPP
#define RADAR_OBS_PUBLISHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <fstream>
#include <list>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <random>

#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include "ulisse_msgs/msg/bounding_box.hpp"
#include "ctrl_toolbox/HelperFunctions.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iomanip>
#include <cstdlib>
#include <libconfig.h++>

//using namespace std::chrono_literals;

libconfig::Config cfg_;
ctb::LatLong centroid; // = {44.0956, 9.8631}; // La Spezia coordinates
double obs_pub_rate;
/*
double n_obs = 1;
double area_size = 40;
double MAX_OBS_SPEED = 2;
double obs_size_x_min = 20;
double obs_size_x_max = 100;
// obs_size_y = obs_size_x / 3*/

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



#endif //RADAR_OBS_PUBLISHER_HPP
