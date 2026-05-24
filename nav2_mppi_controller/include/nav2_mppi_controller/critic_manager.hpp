// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <xtensor/xtensor.hpp>

#include "nav2_msgs/msg/critics_stats.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;

  /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
  virtual ~CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>, ParametersHandler *);

  /**
    * @brief Score trajectories by the set of loaded critic functions.
    *        Also publishes per-critic cost stats if publish_critics_stats_ is true.
    * @param data Struct of necessary information to pass to the critic functions
    */
  void evalTrajectoriesScores(CriticData & data);

protected:
  /**
    * @brief Get parameters (critics to load)
    */
  void getParams();

  /**
    * @brief Load the critic plugins
    */
  virtual void loadCritics();

  /**
    * @brief Get full-name namespaced critic IDs
    */
  std::string getFullName(const std::string & name);

  /**
    * @brief Publish enriched per-critic cost statistics on ~/critics_stats topic
    * @param costs_per_critic     Vector of summed cost deltas, one per critic
    * @param triggered_per_critic Vector of bools, true if critic added any cost
    * @param weights_per_critic   Vector of each critic's primary weight (for normalization)
    */
  void publishCriticsStats(
    const std::vector<double> & costs_per_critic,
    const std::vector<bool> & triggered_per_critic,
    const std::vector<double> & weights_per_critic);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  ParametersHandler * parameters_handler_;
  std::vector<std::string> critic_names_;
  std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  // CriticsStats publisher (Jazzy backport)
  rclcpp::Publisher<nav2_msgs::msg::CriticsStats>::SharedPtr critics_stats_pub_;
  bool publish_critics_stats_{false};

  // Rolling window for trigger_rate computation (one deque per critic)
  static constexpr size_t kTriggerWindow{100};
  std::vector<std::deque<bool>> trigger_history_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_