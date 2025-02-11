// Copyright (c) 2019 Intel Corporation
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

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_ACTION_SERVER_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_ACTION_SERVER_HPP_

#include <chrono>      // NOLINT
#include <future>      // NOLINT
#include <memory>
#include <mutex>       // NOLINT
#include <string>
#include <thread>      // NOLINT

#include "psdk_wrapper/utils/node_thread.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace utils
{
/**
 * @class utils::ActionServer
 * @brief An action server wrapper to make applications simpler using Actions
 *
 * In ROS 2 Humble the executor API allowed you to add a callback group to an executor.
 * In Foxy you must add the entire node to the executor. This conversion passes along
 * the node pointer so that when a dedicated spin thread is requested, a callback group
 * is created using the node API and then the node is added to the executor.
 */
template <typename ActionT>
class ActionServer
{
 public:
  // Callback function to complete main work.
  typedef std::function<void()> ExecuteCallback;
  // Callback function to notify the user that an exception was caught.
  typedef std::function<void()> CompletionCallback;

  /**
   * @brief Templated constructor that takes a node pointer.
   * @param node Shared pointer to the node.
   * @param action_name Name of the action.
   * @param execute_callback Execution callback.
   * @param completion_callback Completion callback.
   * @param server_timeout Timeout to react to stop or preemption requests.
   * @param spin_thread Whether to spin with a dedicated thread.
   * @param options Options to pass to the underlying rcl_action_server_t.
   */
  template <typename NodeT>
  explicit ActionServer(
      NodeT node, const std::string & action_name,
      ExecuteCallback execute_callback,
      CompletionCallback completion_callback = nullptr,
      std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
      bool spin_thread = false,
      const rcl_action_server_options_t & options =
          rcl_action_server_get_default_options())
      : ActionServer(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), node->get_node_waitables_interface(),
            action_name, execute_callback, completion_callback, server_timeout,
            spin_thread, options, node)
  {
  }

  /**
   * @brief Constructor that takes node interfaces and an optional node pointer.
   * @param node_base_interface Node base interface.
   * @param node_clock_interface Node clock interface.
   * @param node_logging_interface Node logging interface.
   * @param node_waitables_interface Node waitables interface.
   * @param action_name Name of the action.
   * @param execute_callback Execution callback.
   * @param completion_callback Completion callback.
   * @param server_timeout Timeout duration.
   * @param spin_thread Whether to spin with a dedicated thread.
   * @param options Options to pass to the underlying rcl_action_server_t.
   * @param node_ptr (Optional) Shared pointer to the node.
   *
   * @note In Foxy, if spin_thread is true then the node pointer is required so that a
   * dedicated callback group can be created and the node can be added to an internal
   * executor.
   */
  explicit ActionServer(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
      rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
      const std::string & action_name, ExecuteCallback execute_callback,
      CompletionCallback completion_callback = nullptr,
      std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
      bool spin_thread = false,
      const rcl_action_server_options_t & options =
          rcl_action_server_get_default_options(),
      rclcpp::Node::SharedPtr node_ptr = nullptr)
      : node_base_interface_(node_base_interface),
        node_clock_interface_(node_clock_interface),
        node_logging_interface_(node_logging_interface),
        node_waitables_interface_(node_waitables_interface),
        action_name_(action_name),
        execute_callback_(execute_callback),
        completion_callback_(completion_callback),
        server_timeout_(server_timeout),
        spin_thread_(spin_thread),
        node_ptr_(node_ptr)
  {
    using namespace std::placeholders;
    // In Foxy the callback group is created via the Node API.
    if (spin_thread_) {
      if (!node_ptr_) {
        throw std::runtime_error(
            "Spin thread enabled but node pointer is null. "
            "In Foxy a valid node is required to create a callback group.");
      }
      callback_group_ = node_ptr_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    action_server_ = rclcpp_action::create_server<ActionT>(
        node_base_interface_, node_clock_interface_, node_logging_interface_,
        node_waitables_interface_, action_name_,
        std::bind(&ActionServer::handle_goal, this, _1, _2),
        std::bind(&ActionServer::handle_cancel, this, _1),
        std::bind(&ActionServer::handle_accepted, this, _1), options,
        callback_group_);
    if (spin_thread_) {
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      // In Foxy, add the entire node to the executor (not just the callback group)
      executor_->add_node(node_ptr_);
      executor_thread_ = std::make_unique<utils::NodeThread>(executor_);
    }
  }

  /**
   * @brief Handle goal request: always accepts if the server is active.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
              std::shared_ptr<const typename ActionT::Goal> /*goal*/)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!server_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    debug_msg("Received request for goal acceptance");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle cancel request: accepts if the handle is active.
   */
  rclcpp_action::CancelResponse
  handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!handle->is_active()) {
      warn_msg(
          "Received request for goal cancellation, but the handle is inactive, so reject the request");
      return rclcpp_action::CancelResponse::REJECT;
    }
    debug_msg("Received request for goal cancellation");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handles accepted goals and either starts execution or queues a pending goal.
   */
  void
  handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    debug_msg("Receiving a new goal");
    if (is_active(current_handle_) || is_running()) {
      debug_msg("An older goal is active, moving the new goal to a pending slot.");
      if (is_active(pending_handle_)) {
        debug_msg("The pending slot is occupied. The previous pending goal will be terminated and replaced.");
        terminate(pending_handle_);
      }
      pending_handle_ = handle;
      preempt_requested_ = true;
    } else {
      if (is_active(pending_handle_)) {
        // Shouldn't reach a state with a pending goal but no current one.
        error_msg("Forgot to handle a preemption. Terminating the pending goal.");
        terminate(pending_handle_);
        preempt_requested_ = false;
      }
      current_handle_ = handle;
      debug_msg("Executing goal asynchronously.");
      execution_future_ = std::async(std::launch::async, [this]() { work(); });
    }
  }

  /**
   * @brief Performs the work associated with the action.
   */
  void
  work()
  {
    while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
      debug_msg("Executing the goal...");
      try {
        execute_callback_();
      } catch (std::exception & ex) {
        RCLCPP_ERROR(
            node_logging_interface_->get_logger(),
            "Action server failed while executing action callback: \"%s\"",
            ex.what());
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        return;
      }
      debug_msg("Blocking processing of new goal handles.");
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      if (stop_execution_) {
        warn_msg("Stopping the thread per request.");
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        break;
      }
      if (is_active(current_handle_)) {
        warn_msg("Current goal was not completed successfully.");
        terminate(current_handle_);
        if (completion_callback_) {
          completion_callback_();
        }
      }
      if (is_active(pending_handle_)) {
        debug_msg("Executing a pending handle on the existing thread.");
        accept_pending_goal();
      } else {
        debug_msg("Done processing available goals.");
        break;
      }
    }
    debug_msg("Worker thread done.");
  }

  /**
   * @brief Activate the action server.
   */
  void
  activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

  /**
   * @brief Deactivate the action server.
   */
  void
  deactivate()
  {
    debug_msg("Deactivating...");
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }
    if (!execution_future_.valid()) {
      return;
    }
    if (is_running()) {
      warn_msg("Requested to deactivate server but goal is still executing. Should check if action server is running before deactivating.");
    }
    using namespace std::chrono;
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      info_msg("Waiting for async process to finish.");
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        throw std::runtime_error("Action callback is still running and missed deadline to stop");
      }
    }
    debug_msg("Deactivation completed.");
  }

  /**
   * @brief Whether the action server is currently executing a goal.
   */
  bool
  is_running()
  {
    return execution_future_.valid() &&
           (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::timeout);
  }

  /**
   * @brief Whether the action server is active.
   */
  bool
  is_server_active()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }

  /**
   * @brief Whether a preemption request is active.
   */
  bool
  is_preempt_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  /**
   * @brief Accept a pending goal.
   */
  const std::shared_ptr<const typename ActionT::Goal>
  accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }
    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      debug_msg("Cancelling the previous goal");
      current_handle_->abort(empty_result());
    }
    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;
    debug_msg("Preempted goal");
    return current_handle_->get_goal();
  }

  /**
   * @brief Terminate the pending goal.
   */
  void
  terminate_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to terminate pending goal when not available");
      return;
    }
    terminate(pending_handle_);
    preempt_requested_ = false;
    debug_msg("Pending goal terminated");
  }

  /**
   * @brief Get the current goal.
   */
  const std::shared_ptr<const typename ActionT::Goal>
  get_current_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return std::shared_ptr<const typename ActionT::Goal>();
    }
    return current_handle_->get_goal();
  }

  /**
   * @brief Get the pending goal.
   */
  const std::shared_ptr<const typename ActionT::Goal>
  get_pending_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }
    return pending_handle_->get_goal();
  }

  /**
   * @brief Whether a cancel request has been made.
   */
  bool
  is_cancel_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (current_handle_ == nullptr) {
      error_msg("Checking for cancel but current goal is not available");
      return false;
    }
    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }
    return current_handle_->is_canceling();
  }

  /**
   * @brief Terminate all active and pending goals.
   */
  void
  terminate_all(typename std::shared_ptr<typename ActionT::Result> result =
                    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

  /**
   * @brief Terminate the current goal.
   */
  void
  terminate_current(typename std::shared_ptr<typename ActionT::Result> result =
                        std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
  }

  /**
   * @brief Report success for the current goal.
   */
  void
  succeeded_current(typename std::shared_ptr<typename ActionT::Result> result =
                        std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (is_active(current_handle_)) {
      debug_msg("Setting succeed on current goal.");
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  /**
   * @brief Publish feedback to clients.
   */
  void
  publish_feedback(
      typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      error_msg("Trying to publish feedback when the current goal handle is not active");
      return;
    }
    current_handle_->publish_feedback(feedback);
  }

 protected:
  // Node interfaces (from the underlying node)
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;

  ExecuteCallback execute_callback_;
  CompletionCallback completion_callback_;
  std::future<void> execution_future_;
  bool stop_execution_{false};

  mutable std::recursive_mutex update_mutex_;
  bool server_active_{false};
  bool preempt_requested_{false};
  std::chrono::milliseconds server_timeout_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  bool spin_thread_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<utils::NodeThread> executor_thread_;

  // In Foxy we store the node pointer to create callback groups and add to the executor.
  rclcpp::Node::SharedPtr node_ptr_;

  /**
   * @brief Generate an empty result object for an action type.
   */
  constexpr auto empty_result() const
  {
    return std::make_shared<typename ActionT::Result>();
  }

  /**
   * @brief Check whether a given goal handle is active.
   */
  constexpr bool
  is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }

  /**
   * @brief Terminate a goal handle with a given result.
   */
  void
  terminate(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
            typename std::shared_ptr<typename ActionT::Result> result =
                std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (is_active(handle)) {
      if (handle->is_canceling()) {
        warn_msg("Client requested to cancel the goal. Cancelling.");
        handle->canceled(result);
      } else {
        warn_msg("Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
    }
  }

  /**
   * @brief Log an informational message.
   */
  void
  info_msg(const std::string & msg) const
  {
    RCLCPP_INFO(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s",
                action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Log a debug message.
   */
  void
  debug_msg(const std::string & msg) const
  {
    RCLCPP_DEBUG(node_logging_interface_->get_logger(),
                 "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Log an error message.
   */
  void
  error_msg(const std::string & msg) const
  {
    RCLCPP_ERROR(node_logging_interface_->get_logger(),
                 "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Log a warning message.
   */
  void
  warn_msg(const std::string & msg) const
  {
    RCLCPP_WARN(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s",
                action_name_.c_str(), msg.c_str());
  }
};

}  // namespace utils

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_ACTION_SERVER_HPP_