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

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();

  // Read publish_critics_stats param and create publisher if enabled
  auto getParam = param_handler->getParamGetter(name);
  getParam(publish_critics_stats_, "publish_critics_stats", false);

  if (publish_critics_stats_) {
    critics_stats_pub_ = node->create_publisher<nav2_msgs::msg::CriticsStats>(
      "~/critics_stats", rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(logger_, "CriticsStats publisher enabled on ~/critics_stats");
  }
}

void CriticManager::getParams()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(CriticData & data)
{
  // Pre-allocate per-critic tracking vectors (only when publishing stats)
  std::vector<double> costs_per_critic;
  std::vector<bool> triggered_per_critic;
  std::vector<double> weights_per_critic;

  if (publish_critics_stats_) {
    costs_per_critic.resize(critics_.size(), 0.0);
    triggered_per_critic.resize(critics_.size(), false);
    weights_per_critic.resize(critics_.size(), 1.0);

    // Initialise trigger history deques on first call
    if (trigger_history_.empty()) {
      trigger_history_.resize(critics_.size());
    }
  }

  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }

    if (publish_critics_stats_) {
      // Snapshot total cost across all trajectories BEFORE this critic runs
      double cost_before = xt::sum(data.costs)();

      critics_[q]->score(data);

      // Compute cost delta added by this critic across all batch trajectories
      double cost_after = xt::sum(data.costs)();
      double delta = cost_after - cost_before;

      costs_per_critic[q]    = delta;
      triggered_per_critic[q] = (delta > 1e-6);
      weights_per_critic[q]   = critics_[q]->getWeight();

      // Update rolling trigger history
      trigger_history_[q].push_back(triggered_per_critic[q]);
      if (trigger_history_[q].size() > kTriggerWindow) {
        trigger_history_[q].pop_front();
      }
    } else {
      critics_[q]->score(data);
    }
  }

  if (publish_critics_stats_ && critics_stats_pub_) {
    publishCriticsStats(costs_per_critic, triggered_per_critic, weights_per_critic);
  }
}

void CriticManager::publishCriticsStats(
  const std::vector<double> & costs_per_critic,
  const std::vector<bool> & triggered_per_critic,
  const std::vector<double> & weights_per_critic)
{
  auto node = parent_.lock();
  if (!node) {
    return;
  }

  nav2_msgs::msg::CriticsStats msg;
  msg.header.stamp    = node->now();
  msg.header.frame_id = "base_link";  // stats are not frame-specific

  // --- total_cost: sum of all critics' deltas this cycle ---
  double total_cost = 0.0;
  for (double c : costs_per_critic) {
    total_cost += c;
  }
  msg.total_cost = total_cost;

  // --- dominant_critic: critic with the highest cost_percentage ---
  size_t dominant_idx = 0;
  double dominant_val = -1.0;

  for (size_t i = 0; i < critics_.size(); i++) {
    msg.critics_names.push_back(critic_names_[i]);
    msg.costs_sum.push_back(costs_per_critic[i]);
    msg.critics_triggered.push_back(triggered_per_critic[i]);

    // cost_percentage: fraction of total this cycle (0–100)
    double pct = (total_cost > 1e-9) ? (costs_per_critic[i] / total_cost * 100.0) : 0.0;
    msg.cost_percentage.push_back(pct);

    // cost_normalized: situation severity stripped of weight tuning
    double w = (weights_per_critic[i] > 1e-9) ? weights_per_critic[i] : 1.0;
    msg.cost_normalized.push_back(costs_per_critic[i] / w);

    // trigger_rate: rolling fraction of cycles this critic fired
    double rate = 0.0;
    if (!trigger_history_.empty() && i < trigger_history_.size() &&
      !trigger_history_[i].empty())
    {
      size_t fired = 0;
      for (bool v : trigger_history_[i]) {fired += v ? 1u : 0u;}
      rate = static_cast<double>(fired) / static_cast<double>(trigger_history_[i].size());
    }
    msg.trigger_rate.push_back(rate);

    if (pct > dominant_val) {
      dominant_val = pct;
      dominant_idx = i;
    }
  }

  msg.dominant_critic = critics_.size() > 0 ? critic_names_[dominant_idx] : "";

  critics_stats_pub_->publish(msg);
}

}  // namespace mppi