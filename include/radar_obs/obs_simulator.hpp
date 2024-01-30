#ifndef RADAR_OBS_OBSSIMULATOR_HPP
#define RADAR_OBS_OBSSIMULATOR_HPP

#include "radar_obs/publisher.hpp"
#include "radar_obs/utils.hpp"
#include <eigen3/Eigen/Eigen>
#include <utility>

class obs_simulator {
public:
    std::string id;
    ctb::LatLong position;
    double heading;
    double bb_x, bb_y;
    double speed;
    double spawn_time = 0;
    double kill_time = 99999;


    ulisse_msgs::msg::BoundingBox bb_max;
    ulisse_msgs::msg::BoundingBox bb_safe;
    double gap;

    // from file
    obs_simulator(std::string id, double spawn_time, double kill_time, double latitude, double longitude, double heading, double speed,
                  double dim_x, double dim_y, ulisse_msgs::msg::BoundingBox bb_max, ulisse_msgs::msg::BoundingBox bb_safe, double gap):
            id(std::move(id)), heading(heading), bb_x(dim_x), bb_y(dim_y), speed(speed), spawn_time(spawn_time), kill_time(kill_time), bb_max(bb_max), bb_safe(bb_safe), gap(gap)
    {
      position = GetLatLong(Eigen::Vector2d(latitude, longitude));
    }

    void print(){
      std::cout<<
        "id:" << id <<
        " time:" << spawn_time  <<
        " speed: " << speed <<
        " dim y: " << bb_y <<
        " bb_max_x_bow: " << bb_max.x_bow_ratio <<
        std::endl;
    }

    // TODO OLD, HAS TO BE UPDATED
    /*// random gen
    obs_simulator(int i){
      std::random_device r;
      std::default_random_engine e1(r());
      std::uniform_real_distribution<double> pos_gen(-area_size/2, area_size/2);
      std::uniform_real_distribution<double> speed_gen(0, MAX_OBS_SPEED);
      std::uniform_real_distribution<double> heading_gen(-M_PI, M_PI);
      std::uniform_real_distribution<double> dim_x_gen(obs_size_x_min, obs_size_x_max);

      id = std::to_string(i);
      speed = speed_gen(e1);
      //heading = heading_gen(e1);
      heading = 2;
      position = GetLatLong(Eigen::Vector2d(pos_gen(e1), pos_gen(e1)));
      bb_x = dim_x_gen(e1);
      bb_y = bb_x/3;
    }*/

    ctb::LatLong GetPosition(double time) const{
      Eigen::Vector2d pos = GetLocal(position);
      pos.x() += speed * time * cos(heading);
      pos.y() += speed * time * sin(heading);
      return GetLatLong(pos);
    }

    void GetStatus(ulisse_msgs::msg::Obstacle& msg, double d_time) const{
      ctb::LatLong pos = GetPosition(d_time - spawn_time);
      msg.id = id;
      msg.center.latitude = pos.latitude;
      msg.center.longitude = pos.longitude;
      msg.heading = heading;
      msg.b_box_dim_x = bb_x;
      msg.b_box_dim_y = bb_y;

      msg.bb_max = bb_max;
      msg.bb_safe = bb_safe;
      msg.uncertainty_gap = gap;

      msg.vel_x = speed * cos(heading);
      msg.vel_y = speed * sin(heading);
      msg.high_priority = false;
      msg.show_id = true;
      msg.color.r=0;
      msg.color.g=0;
      msg.color.b=0;
    }

};





#endif //RADAR_OBS_OBSSIMULATOR_HPP
