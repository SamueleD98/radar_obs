#include "radar_obs/config.hpp"
#include "radar_obs/obs_simulator.hpp"

auto timer_d = 2000ms;

class ObsPublisher : public rclcpp::Node {
public:
  ObsPublisher() : Node("radar") {
    starting_time_ = rclcpp::Clock().now();

    if (false) {
      // Random Gen
      for (int obs_id = 0; obs_id < n_obs; obs_id++) {
        obstacles.push_back(obs_simulator(obs_id));
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Reading data..");
      // From file
      std::ifstream obs_text;
      obs_text.open("src/radar_obs/files/obstacles.txt");
      //if (myfile.is_open()) {
      std::string obs_line;
      std::getline(obs_text, obs_line); // remove first line
      while (std::getline(obs_text, obs_line)) {
        if (obs_line == "---") break;
        std::string id;
        double time, latitude, longitude, heading, speed, dim_x, dim_y;
        std::stringstream ss(obs_line);
        ss >> time;
        ss >> id;
        ss >> latitude;
        ss >> longitude;
        ss >> heading;
        ss >> speed;
        ss >> dim_x;
        ss >> dim_y;
        obstacles.push_back(
                obs_simulator(id, time, latitude, longitude, heading, speed, dim_x, dim_y));
      }
      obs_text.close();
    }

    publisher_ = this->create_publisher<ulisse_msgs::msg::Obstacle>("/ulisse/ctrl/obstacle", 10);
    timer_ = this->create_wall_timer(timer_d, std::bind(&ObsPublisher::timer_callback, this));
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
      if (it->time <= current_time) {
        // Find the obstacle in tracking_obs and erase it if it exists
        auto tracking_it = std::find_if(tracking_obs.begin(), tracking_obs.end(),
                                        [&it](const auto &entry) { return entry.id == it->id; });
        if (tracking_it != tracking_obs.end()) {
          ctb::LatLong pos = tracking_it->GetPosition(current_time);
          tracking_obs.erase(tracking_it);
          it->position = pos;
        }
        // Add the obstacle to tracking_obs and remove it from obstacles
        tracking_obs.push_back(*it);
        it = obstacles.erase(it);
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