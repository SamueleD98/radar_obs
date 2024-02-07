#include "radar_obs/publisher.hpp"
#include "radar_obs/obs_simulator.hpp"

class ObsPublisher : public rclcpp::Node {
public:
  ObsPublisher() : Node("radar") {
    starting_time_ = rclcpp::Clock().now();
    RCLCPP_INFO(this->get_logger(), "Loading configuration..");

    std::string radar_dir = ament_index_cpp::get_package_share_directory("radar_obs");
    std::string radar_dir_conf = radar_dir;
    radar_dir_conf.append("/conf/configuration.cfg");
    cfg_.readFile(radar_dir_conf.c_str());

    centroid.latitude = cfg_.lookup("centroid.latitude");
    centroid.longitude = cfg_.lookup("centroid.longitude");
    obs_pub_rate = cfg_.lookup("obs_pub_rate");
    std::string scenario = cfg_.lookup("scenario");

    std::string radar_dir_scenario = radar_dir;
    radar_dir_scenario.append("/files/scenario_");
    radar_dir_scenario.append(scenario);
    radar_dir_scenario.append(".cfg");
    libconfig::Config scenario_cfg;
    std::cout<<"trying to load conf at: "<<radar_dir_scenario<<std::endl;
    scenario_cfg.readFile(radar_dir_scenario.c_str());
    const libconfig::Setting &root = scenario_cfg.getRoot();

    RCLCPP_INFO(this->get_logger(), "..Done");

    const libconfig::Setting &obss = root["obstacles"];
    int count = obss.getLength();
    for (int i = 0; i < count; ++i) {
      const libconfig::Setting &obs = obss[i];

      std::string id;
      double spawn_time, kill_time, lat, lon, heading, speed, dim_x, dim_y, gap, err_loc, err_velocity;
      ulisse_msgs::msg::BoundingBox bb_max, bb_safe;

      if (!obs.lookup("active")) continue;

      obs.lookupValue("id", id);
      obs.lookupValue("spawn_time", spawn_time);
      obs.lookupValue("kill_time", kill_time);
      /*lat = obs.lookup("position.N");
      lon = obs.lookup("position.E");*/
      obs.lookupValue("heading", heading);
      obs.lookupValue("speed", speed);

      dim_x = obs.lookup("dim.x");
      dim_y = obs.lookup("dim.y");

      bb_max.x_bow_ratio = obs.lookup("bbs.bb_max.x_bow_ratio");
      bb_max.x_stern_ratio = obs.lookup("bbs.bb_max.x_stern_ratio");
      bb_max.y_starboard_ratio = obs.lookup("bbs.bb_max.y_starboard_ratio");
      bb_max.y_port_ratio = obs.lookup("bbs.bb_max.y_port_ratio");
      bb_safe.x_bow_ratio = obs.lookup("bbs.bb_safe.x_bow_ratio");
      bb_safe.x_stern_ratio = obs.lookup("bbs.bb_safe.x_stern_ratio");
      bb_safe.y_starboard_ratio = obs.lookup("bbs.bb_safe.y_starboard_ratio");
      bb_safe.y_port_ratio = obs.lookup("bbs.bb_safe.y_port_ratio");

      obs.lookupValue("gap", gap);

      err_loc = obs.lookup("errors.localization");
      err_velocity = obs.lookup("errors.velocity");

      try {
        lat = obs.lookup("position.N");
        lon = obs.lookup("position.E");
      }
      catch (...) {
        Eigen::Vector2d pos_NED = GetLocal(ctb::LatLong(obs.lookup("position.lat"), obs.lookup("position.long")));
        lat = pos_NED.x();
        lon = pos_NED.y();
      }

      obstacles.emplace_back(id, spawn_time, kill_time, lat, lon, heading, speed, dim_x, dim_y, bb_max, bb_safe, gap, err_loc, err_velocity);
      //obstacles.at(obstacles.size()-1).print();
    }

    publisher_ = this->create_publisher<ulisse_msgs::msg::Obstacle>("/ulisse/ctrl/obstacle", 10);
    auto duration = std::chrono::duration<double>(obs_pub_rate);
    timer_ = this->create_wall_timer(duration, std::bind(&ObsPublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Starting..");
  }

private:

  rclcpp::Time starting_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ulisse_msgs::msg::Obstacle>::SharedPtr publisher_;
  std::vector <obs_simulator> tracking_obs;
  std::vector <obs_simulator> obstacles;

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), ".");
    auto current_time = rclcpp::Clock().now().seconds() - starting_time_.seconds();
    // Iterate through obstacles and check if they should be moved to tracking_obs

    for (auto it = obstacles.begin(); it != obstacles.end(); /* nothing on purpose */) {
      if (it->spawn_time <= current_time) {
        // Find the obstacle in tracking_obs and erase it if it exists
        auto tracking_it = std::find_if(tracking_obs.begin(), tracking_obs.end(),
                                        [&it](const auto &entry) { return entry.id == it->id; });
        if (tracking_it != tracking_obs.end()) {
          //ctb::LatLong pos = tracking_it->GetPosition(current_time);
          tracking_obs.erase(tracking_it);
          //it->position = pos;
        }
        // Check if kill_time is less than current_time and erase from obstacles
        if (it->kill_time <= current_time) {
          it = obstacles.erase(it);
        } else {
          // Add the obstacle to tracking_obs and remove it from obstacles
          tracking_obs.push_back(*it);
          ++it;
        }
      } else {
        ++it;
      }
    }
    // Publish the status of obstacles in tracking_obs
    for (auto &obs: tracking_obs) {
      ulisse_msgs::msg::Obstacle message;
      obs.GetStatus(message, current_time);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.id.c_str());
      publisher_->publish(message);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObsPublisher>());
  rclcpp::shutdown();
  return 0;
}