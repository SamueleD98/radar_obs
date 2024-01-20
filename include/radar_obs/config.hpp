#ifndef RADAR_OBS_CONFIG_HPP
#define RADAR_OBS_CONFIG_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <fstream>
#include <list>
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include <random>
#include "ctrl_toolbox/HelperFunctions.h"
#include <eigen3/Eigen/Eigen>
#include <fstream>

using namespace std::chrono_literals;

// Generation params
//ctb::LatLong centroid(44.09568491, 9.86398815);
ctb::LatLong centroid = {44.0956, 9.8631}; // La Spezia coordinates
double n_obs = 1;
double area_size = 40;
double MAX_OBS_SPEED = 2;
double obs_size_x_min = 20;
double obs_size_x_max = 100;
// obs_size_y = obs_size_x / 3

#endif //RADAR_OBS_CONFIG_HPP
