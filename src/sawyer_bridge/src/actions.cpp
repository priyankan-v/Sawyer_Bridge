// SPDX-License-Identifier: MIT
// ROS 1 actions use five topics; ROS 2 actions use goal/cancel/result services
// plus feedback/status topics. These adapters preserve the action lifecycle.
#include "sawyer_bridge/actions.hpp"
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <intera_core_msgs/CalibrationCommandActionGoal.h>
#include <intera_core_msgs/CalibrationCommandActionFeedback.h>
#include <intera_core_msgs/CalibrationCommandActionResult.h>
#include <intera_core_msgs/action/calibration_command.hpp>
#include <intera_motion_msgs/MotionCommandActionGoal.h>
#include <intera_motion_msgs/MotionCommandActionFeedback.h>
#include <intera_motion_msgs/MotionCommandActionResult.h>
#include <intera_motion_msgs/action/motion_command.hpp>
#include <intera_motion_msgs/Trajectory.h>
#include <intera_motion_msgs/TrajectoryAnalysis.h>
#include <intera_motion_msgs/MotionStatus.h>
#include <ros1_bridge/factory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace sawyer_bridge {
using Clock = std::chrono::steady_clock;
using Status = actionlib_msgs::GoalStatus;

struct Calibration {
  using Action = intera_core_msgs::action::CalibrationCommand;
  using Goal = intera_core_msgs::CalibrationCommandActionGoal;
  using Feedback = intera_core_msgs::CalibrationCommandActionFeedback;
  using Result = intera_core_msgs::CalibrationCommandActionResult;
  static void to1(const Action::Goal & a, Goal & b) { b.goal.command = a.command; }
  static void to2(const Goal & a, Action::Goal & b) { b.command = a.goal.command; }
  static void to1(const Action::Feedback & a, Feedback & b) {
    b.feedback.currentState = a.current_state; b.feedback.numberOfPoses = a.number_of_poses;
    b.feedback.currentPoseNumber = a.current_pose_number;
  }
  static void to2(const Feedback & a, Action::Feedback & b) {
    b.current_state = a.feedback.currentState; b.number_of_poses = a.feedback.numberOfPoses;
    b.current_pose_number = a.feedback.currentPoseNumber;
  }
  static void to1(const Action::Result & a, Result & b) {
    b.result.result = a.result; b.result.statusId = a.status_id;
  }
  static void to2(const Result & a, Action::Result & b) {
    b.result = a.result.result; b.status_id = a.result.statusId;
  }
  static void error(Action::Result & a, const std::string & text) { a.result = false; a.status_id = text; }
};
struct Motion {
  using Action = intera_motion_msgs::action::MotionCommand;
  using Goal = intera_motion_msgs::MotionCommandActionGoal;
  using Feedback = intera_motion_msgs::MotionCommandActionFeedback;
  using Result = intera_motion_msgs::MotionCommandActionResult;
  template<class One, class Two> using Convert = ros1_bridge::Factory<One, Two>;
  static void to1(const Action::Goal & a, Goal & b) {
    b.goal.command = a.command;
    Convert<intera_motion_msgs::Trajectory, intera_motion_msgs::msg::Trajectory>::convert_2_to_1(a.trajectory, b.goal.trajectory);
  }
  static void to2(const Goal & a, Action::Goal & b) {
    b.command = a.goal.command;
    Convert<intera_motion_msgs::Trajectory, intera_motion_msgs::msg::Trajectory>::convert_1_to_2(a.goal.trajectory, b.trajectory);
  }
  static void to1(const Action::Feedback & a, Feedback & b) {
    Convert<intera_motion_msgs::MotionStatus, intera_motion_msgs::msg::MotionStatus>::convert_2_to_1(a.status, b.feedback.status);
  }
  static void to2(const Feedback & a, Action::Feedback & b) {
    Convert<intera_motion_msgs::MotionStatus, intera_motion_msgs::msg::MotionStatus>::convert_1_to_2(a.feedback.status, b.status);
  }
  static void to1(const Action::Result & a, Result & b) {
    b.result.result = a.result; b.result.errorId = a.error_id;
    b.result.last_successful_waypoint = a.last_successful_waypoint;
    Convert<intera_motion_msgs::TrajectoryAnalysis, intera_motion_msgs::msg::TrajectoryAnalysis>::convert_2_to_1(a.trajectory_analysis, b.result.trajectory_analysis);
  }
  static void to2(const Result & a, Action::Result & b) {
    b.result = a.result.result; b.error_id = a.result.errorId;
    b.last_successful_waypoint = a.result.last_successful_waypoint;
    Convert<intera_motion_msgs::TrajectoryAnalysis, intera_motion_msgs::msg::TrajectoryAnalysis>::convert_1_to_2(a.result.trajectory_analysis, b.trajectory_analysis);
  }
  static void error(Action::Result & a, const std::string & text) { a.result = false; a.error_id = text; }
};

std::string id_string(const rclcpp_action::GoalUUID & uuid) {
  std::ostringstream stream; stream << "sawyer_bridge_" << std::hex << std::setfill('0');
  for (auto byte : uuid) { stream << std::setw(2) << static_cast<unsigned>(byte); }
  return stream.str();
}
bool matches_cancel(const actionlib_msgs::GoalID & request, const actionlib_msgs::GoalID & goal) {
  return (!request.id.empty() && request.id == goal.id) ||
    (request.id.empty() && request.stamp.isZero()) ||
    (!request.stamp.isZero() && goal.stamp <= request.stamp);
}

// ROS 2 clients -> existing ROS 1 action server.
template<class Traits> class ToRos1 : public ActionBridge {
  using Action = typename Traits::Action;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Goal = typename Traits::Goal;
  using Feedback = typename Traits::Feedback;
  using Result = typename Traits::Result;
  struct Pending {
    std::shared_ptr<Handle> handle;
    typename Feedback::ConstPtr feedback;
    typename Result::ConstPtr result;
    Clock::time_point seen = Clock::now();
  };
  std::mutex mutex_;
  std::map<std::string, Pending> pending_;
  Clock::time_point heartbeat_ = Clock::now();
  double timeout_;
  rclcpp::Node::SharedPtr node_;
  ros::Publisher goals_, cancels_;
  ros::Subscriber feedback_, results_, status_;
  typename rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
public:
  ToRos1(ros::NodeHandle & ros1, rclcpp::Node::SharedPtr node, const std::string & name, double timeout)
  : timeout_(timeout), node_(node) {
    goals_ = ros1.advertise<Goal>(name + "/goal", 20);
    cancels_ = ros1.advertise<actionlib_msgs::GoalID>(name + "/cancel", 20);
    feedback_ = ros1.subscribe<Feedback>(name + "/feedback", 20,
      [this](const typename Feedback::ConstPtr & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto found = pending_.find(msg->status.goal_id.id);
        if (found != pending_.end()) { found->second.feedback = msg; }
      });
    results_ = ros1.subscribe<Result>(name + "/result", 20,
      [this](const typename Result::ConstPtr & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto found = pending_.find(msg->status.goal_id.id);
        if (found != pending_.end()) { found->second.result = msg; }
      });
    status_ = ros1.subscribe<actionlib_msgs::GoalStatusArray>(name + "/status", 20,
      [this](const actionlib_msgs::GoalStatusArray::ConstPtr & msg) {
        std::lock_guard<std::mutex> lock(mutex_); heartbeat_ = Clock::now();
        for (const auto & status : msg->status_list) {
          auto found = pending_.find(status.goal_id.id);
          if (found != pending_.end()) { found->second.seen = heartbeat_; }
        }
      });
    server_ = rclcpp_action::create_server<Action>(node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename Action::Goal>) {
        std::lock_guard<std::mutex> lock(mutex_);
        return goals_.getNumSubscribers() && status_.getNumPublishers() &&
          std::chrono::duration<double>(Clock::now() - heartbeat_).count() < timeout_ ?
          rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
      },
      [this](std::shared_ptr<Handle> handle) {
        auto id = id_string(handle->get_goal_id());
        { std::lock_guard<std::mutex> lock(mutex_);
          if (!pending_.count(id)) { return rclcpp_action::CancelResponse::REJECT; }
        }
        actionlib_msgs::GoalID cancel; cancel.id = id; cancels_.publish(cancel);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> handle) {
        Goal goal;
        try { Traits::to1(*handle->get_goal(), goal); }
        catch (const std::exception & error) {
          auto result = std::make_shared<typename Action::Result>(); Traits::error(*result, error.what());
          handle->abort(result); return;
        }
        goal.header.stamp = ros::Time::now(); goal.goal_id.stamp = goal.header.stamp;
        goal.goal_id.id = id_string(handle->get_goal_id());
        { std::lock_guard<std::mutex> lock(mutex_); pending_[goal.goal_id.id].handle = handle; }
        goals_.publish(goal);
      });
    // Finish results on the ROS 2 executor so cancellation state transitions
    // complete before a fast ROS 1 cancellation acknowledgement is applied.
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this]() { drain(); });
  }
private:
  void drain() {
    std::vector<Pending> work;
    bool disconnected;
    { std::lock_guard<std::mutex> lock(mutex_);
      disconnected = std::chrono::duration<double>(Clock::now() - heartbeat_).count() > timeout_;
      for (auto it = pending_.begin(); it != pending_.end();) {
        bool lost = disconnected || std::chrono::duration<double>(Clock::now() - it->second.seen).count() > timeout_;
        work.push_back(it->second); it->second.feedback.reset();
        if (it->second.result || lost) { it = pending_.erase(it); } else { ++it; }
      }
    }
    for (auto & pending : work) {
      if (pending.feedback && pending.handle->is_active()) {
        auto feedback = std::make_shared<typename Action::Feedback>();
        Traits::to2(*pending.feedback, *feedback); pending.handle->publish_feedback(feedback);
      }
      if (pending.result) {
        auto result = std::make_shared<typename Action::Result>(); Traits::to2(*pending.result, *result);
        auto status = pending.result->status.status;
        if (status == Status::SUCCEEDED) { pending.handle->succeed(result); }
        else if ((status == Status::PREEMPTED || status == Status::RECALLED) && pending.handle->is_canceling()) {
          pending.handle->canceled(result);
        } else { pending.handle->abort(result); }
      } else if (disconnected || std::chrono::duration<double>(Clock::now() - pending.seen).count() > timeout_) {
        auto result = std::make_shared<typename Action::Result>(); Traits::error(*result, "ROS1_ACTION_SERVER_UNAVAILABLE");
        pending.handle->abort(result);
      }
    }
  }
};

// ROS 1 clients -> existing ROS 2 action server.
template<class Traits> class ToRos2 : public ActionBridge {
  using Action = typename Traits::Action;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Goal = typename Traits::Goal;
  using Feedback = typename Traits::Feedback;
  using Result = typename Traits::Result;
  struct Pending {
    actionlib_msgs::GoalStatus status;
    std::shared_ptr<Handle> handle;
    bool cancel = false;
    Clock::time_point changed = Clock::now();
  };
  std::mutex mutex_;
  std::map<std::string, Pending> pending_;
  double timeout_;
  Clock::time_point last_ready_ = Clock::now();
  ros::Publisher feedback_, results_, status_;
  ros::Subscriber goals_, cancels_;
  typename rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t generated_ = 0;
public:
  ToRos2(ros::NodeHandle & ros1, rclcpp::Node::SharedPtr node, const std::string & name, double timeout)
  : timeout_(timeout) {
    feedback_ = ros1.advertise<Feedback>(name + "/feedback", 20);
    results_ = ros1.advertise<Result>(name + "/result", 20);
    status_ = ros1.advertise<actionlib_msgs::GoalStatusArray>(name + "/status", 20);
    client_ = rclcpp_action::create_client<Action>(node, name);
    goals_ = ros1.subscribe<Goal>(name + "/goal", 20,
      [this](const typename Goal::ConstPtr & msg) { send(*msg); });
    cancels_ = ros1.subscribe<actionlib_msgs::GoalID>(name + "/cancel", 20,
      [this](const actionlib_msgs::GoalID::ConstPtr & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto & pair : pending_) {
          auto & entry = pair.second;
          if (entry.status.status <= Status::ACTIVE && matches_cancel(*msg, entry.status.goal_id)) {
            entry.cancel = true;
            if (entry.handle) { entry.status.status = Status::PREEMPTING; client_->async_cancel_goal(entry.handle); }
          }
        }
      });
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() { heartbeat(); });
  }
private:
  void finish(const std::string & id, uint8_t status, const typename Action::Result & payload) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = pending_.find(id);
    if (found == pending_.end() || (found->second.status.status >= Status::PREEMPTED &&
        found->second.status.status != Status::PREEMPTING)) { return; }
    found->second.status.status = status; found->second.changed = Clock::now();
    Result result; result.header.stamp = ros::Time::now(); result.status = found->second.status;
    Traits::to1(payload, result); results_.publish(result);
  }
  void send(const Goal & message) {
    auto goal = std::make_shared<typename Action::Goal>(); Traits::to2(message, *goal);
    std::string id;
    { std::lock_guard<std::mutex> lock(mutex_);
      id = message.goal_id.id;
      if (id.empty()) { id = "sawyer_bridge_" + std::to_string(ros::WallTime::now().toNSec()) + "_" + std::to_string(++generated_); }
      if (pending_.count(id)) { return; }
      pending_[id].status.goal_id = message.goal_id; pending_[id].status.goal_id.id = id;
      pending_[id].status.status = Status::PENDING;
    }
    if (!client_->action_server_is_ready()) {
      typename Action::Result result; Traits::error(result, "ROS2_ACTION_SERVER_UNAVAILABLE");
      finish(id, Status::REJECTED, result); return;
    }
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, id](std::shared_ptr<Handle> handle) {
      if (!handle) { typename Action::Result result; Traits::error(result, "ROS2_GOAL_REJECTED"); finish(id, Status::REJECTED, result); return; }
      std::lock_guard<std::mutex> lock(mutex_);
      auto found = pending_.find(id);
      if (found == pending_.end() || found->second.status.status != Status::PENDING) { client_->async_cancel_goal(handle); return; }
      found->second.handle = handle; found->second.status.status = found->second.cancel ? Status::PREEMPTING : Status::ACTIVE;
      if (found->second.cancel) { client_->async_cancel_goal(handle); }
    };
    options.feedback_callback = [this, id](std::shared_ptr<Handle>, const std::shared_ptr<const typename Action::Feedback> payload) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto found = pending_.find(id); if (found == pending_.end()) { return; }
      Feedback feedback; feedback.header.stamp = ros::Time::now(); feedback.status = found->second.status;
      Traits::to1(*payload, feedback); feedback_.publish(feedback);
    };
    options.result_callback = [this, id](const typename Handle::WrappedResult & result) {
      auto status = result.code == rclcpp_action::ResultCode::SUCCEEDED ? Status::SUCCEEDED :
        result.code == rclcpp_action::ResultCode::CANCELED ? Status::PREEMPTED : Status::ABORTED;
      if (result.result) { finish(id, status, *result.result); }
      else { typename Action::Result payload; Traits::error(payload, "ROS2_RESULT_UNAVAILABLE"); finish(id, Status::ABORTED, payload); }
    };
    client_->async_send_goal(*goal, options);
  }
  void heartbeat() {
    auto now = Clock::now();
    if (client_->action_server_is_ready()) { last_ready_ = now; }
    actionlib_msgs::GoalStatusArray status; status.header.stamp = ros::Time::now();
    std::vector<std::string> lost;
    { std::lock_guard<std::mutex> lock(mutex_);
      for (auto it = pending_.begin(); it != pending_.end();) {
        auto code = it->second.status.status;
        bool terminal = code >= Status::PREEMPTED && code != Status::PREEMPTING;
        if (!terminal && (std::chrono::duration<double>(now - last_ready_).count() > timeout_ ||
            (!it->second.handle && std::chrono::duration<double>(now - it->second.changed).count() > timeout_))) { lost.push_back(it->first); }
        if (terminal && std::chrono::duration<double>(now - it->second.changed).count() > 5) { it = pending_.erase(it); }
        else { status.status_list.push_back(it->second.status); ++it; }
      }
    }
    status_.publish(status);
    for (auto & id : lost) { typename Action::Result result; Traits::error(result, "ROS2_ACTION_SERVER_UNAVAILABLE"); finish(id, Status::ABORTED, result); }
  }
};

template<class Traits> std::shared_ptr<ActionBridge> make(
  ros::NodeHandle & ros1, rclcpp::Node::SharedPtr node, const std::string & name,
  const std::string & direction, double timeout) {
  if (direction == "2to1") { return std::make_shared<ToRos1<Traits>>(ros1, node, name, timeout); }
  if (direction == "1to2") { return std::make_shared<ToRos2<Traits>>(ros1, node, name, timeout); }
  throw std::runtime_error("Action direction must be 2to1 or 1to2: " + name);
}
std::vector<std::shared_ptr<ActionBridge>> create_action_bridges(ros::NodeHandle & ros1, rclcpp::Node::SharedPtr node, const std::string & parameter) {
  std::vector<std::shared_ptr<ActionBridge>> bridges;
  XmlRpc::XmlRpcValue actions;
  if (!ros1.getParam(parameter, actions)) { return bridges; }
  if (actions.getType() != XmlRpc::XmlRpcValue::TypeArray) { throw std::runtime_error("actions must be a list"); }
  std::map<std::string, bool> names;
  for (int i = 0; i < actions.size(); ++i) {
    auto & entry = actions[i];
    if (entry.hasMember("enabled") && !static_cast<bool>(entry["enabled"])) { continue; }
    std::string name = entry["action"], type = entry["type"], direction = entry["direction"];
    if (names.count(name)) { throw std::runtime_error("Duplicate action endpoint: " + name); }
    names[name] = true;
    double timeout = 5.;
    if (entry.hasMember("server_timeout")) { timeout = entry["server_timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(entry["server_timeout"]) : static_cast<double>(entry["server_timeout"]); }
    if (!(timeout > 0)) { throw std::runtime_error("server_timeout must be positive"); }
    if (type == "intera_core_msgs/action/CalibrationCommand") { bridges.push_back(make<Calibration>(ros1, node, name, direction, timeout)); }
    else if (type == "intera_motion_msgs/action/MotionCommand") { bridges.push_back(make<Motion>(ros1, node, name, direction, timeout)); }
    else { throw std::runtime_error("Unsupported Intera action: " + type); }
    RCLCPP_INFO(node->get_logger(), "Created %s action bridge: %s (%s)", direction.c_str(), name.c_str(), type.c_str());
  }
  return bridges;
}
}
