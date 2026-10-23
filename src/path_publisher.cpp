#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"


class PathPublisher : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath =
        rclcpp_action::ClientGoalHandle<FollowPath>;

    PathPublisher()
        : Node("path_publisher")
    {
        // --------------------------------------------------
        // Parameters
        // --------------------------------------------------

        this->declare_parameter<std::string>(
            "action_name",
            "follow_path"
        );

        this->declare_parameter<std::string>(
            "frame_id",
            "eduard/map"
        );

        this->declare_parameter<double>(
            "point_spacing",
            0.1
        );

        this->declare_parameter<std::string>(
            "controller_id",
            "FollowPath"
        );

        this->declare_parameter<std::string>(
            "goal_checker_id",
            "general_goal_checker"
        );

        this->declare_parameter<std::string>(
            "progress_checker_id",
            "progress_checker"
        );

        // Flat array of x, y, yaw values.
        // Yaw is in radians.

        this->declare_parameter<std::vector<double>>(
            "waypoints",
            {
                0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                1.0, 1.0, M_PI / 2.0,
                0.0, 1.0, M_PI,
                0.0, 0.0, -M_PI / 2.0
            }
        );

        // Retrieve parameters.

        action_name_ =
            this->get_parameter("action_name").as_string();

        frame_id_ =
            this->get_parameter("frame_id").as_string();

        point_spacing_ =
            this->get_parameter("point_spacing").as_double();

        controller_id_ =
            this->get_parameter("controller_id").as_string();

        goal_checker_id_ =
            this->get_parameter("goal_checker_id").as_string();

        progress_checker_id_ =
            this->get_parameter("progress_checker_id").as_string();

        const auto raw_waypoints =
            this->get_parameter("waypoints").as_double_array();

        // Validate parameters.

        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());

        if (
            raw_waypoints.size() < 6 ||
            raw_waypoints.size() % 3 != 0
        ) {
            throw std::invalid_argument(
                "waypoints must contain at least two waypoints, "
                "with each waypoint formatted as x, y, yaw"
            );
        }

        if (point_spacing_ <= 0.0) {
            throw std::invalid_argument(
                "point_spacing must be greater than zero"
            );
        }

        // Convert the flat waypoint array into (x, y, yaw) tuples.

        for (std::size_t i = 0; i < raw_waypoints.size(); i += 3) {
            waypoints_.push_back({
                raw_waypoints[i],
                raw_waypoints[i + 1],
                raw_waypoints[i + 2]
            });
        }

        // --------------------------------------------------
        // Generate the path
        // --------------------------------------------------

        path_ = generate_path(
            waypoints_,
            point_spacing_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Generated path containing %zu poses in frame \"%s\"",
            path_.poses.size(),
            frame_id_.c_str()
        );

        // --------------------------------------------------
        // Path Publisher
        // --------------------------------------------------

        path_publisher_ =
            this->create_publisher<nav_msgs::msg::Path>(
                "/eduard/plan",
                10
            );

        // --------------------------------------------------
        // Create the FollowPath action client
        // --------------------------------------------------

        action_client_ =
            rclcpp_action::create_client<FollowPath>(
                this,
                action_name_
            );

        goal_handle_.reset();
        goal_sent_ = false;

        last_feedback_log_time_ = 0.0;

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for action server: %s",
            action_name_.c_str()
        );

        // Wait for Nav2 without blocking the executor.

        server_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(
                &PathPublisher::try_send_goal,
                this
            )
        );
    }

private:

    // ------------------------------------------------------
    // Types
    // ------------------------------------------------------

    using Waypoint = std::tuple<double, double, double>;


    // ------------------------------------------------------
    // Parameters and member variables
    // ------------------------------------------------------

    std::string action_name_;
    std::string frame_id_;

    double point_spacing_;

    std::string controller_id_;
    std::string goal_checker_id_;
    std::string progress_checker_id_;

    std::vector<Waypoint> waypoints_;

    nav_msgs::msg::Path path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        path_publisher_;

    rclcpp_action::Client<FollowPath>::SharedPtr
        action_client_;

    GoalHandleFollowPath::SharedPtr goal_handle_;

    rclcpp::TimerBase::SharedPtr server_timer_;

    bool goal_sent_;

    double last_feedback_log_time_;


    // ------------------------------------------------------
    // Path generation
    // ------------------------------------------------------

    static double normalize_angle(double angle)
    {
        // Normalize angle to [-pi, pi].

        return std::atan2(
            std::sin(angle),
            std::cos(angle)
        );
    }


    static geometry_msgs::msg::Quaternion yaw_to_quaternion(
        double yaw)
    {
        // Convert planar yaw to a quaternion.

        geometry_msgs::msg::Quaternion q;

        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);

        return q;
    }


    nav_msgs::msg::Path generate_path(
        const std::vector<Waypoint> & waypoints,
        double spacing)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame_id_;

        for (
            std::size_t segment_index = 0;
            segment_index < waypoints.size() - 1;
            ++segment_index
        ) {
            const auto & start = waypoints[segment_index];
            const auto & end = waypoints[segment_index + 1];

            const double x0 = std::get<0>(start);
            const double y0 = std::get<1>(start);
            const double yaw0 = std::get<2>(start);

            const double x1 = std::get<0>(end);
            const double y1 = std::get<1>(end);
            const double yaw1 = std::get<2>(end);

            const double distance = std::hypot(
                x1 - x0,
                y1 - y0
            );

            const int steps = std::max(
                1,
                static_cast<int>(
                    std::ceil(distance / spacing)
                )
            );

            const double yaw_delta = normalize_angle(
                yaw1 - yaw0
            );

            // Include the first point only for the first segment.

            const int first_step =
                (segment_index == 0) ? 0 : 1;

            for (
                int step = first_step;
                step <= steps;
                ++step
            ) {
                const double t =
                    static_cast<double>(step) /
                    static_cast<double>(steps);

                const double x =
                    x0 + t * (x1 - x0);

                const double y =
                    y0 + t * (y1 - y0);

                const double yaw = normalize_angle(
                    yaw0 + t * yaw_delta
                );

                geometry_msgs::msg::PoseStamped pose;

                pose.header.frame_id = frame_id_;

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation =
                    yaw_to_quaternion(yaw);

                path.poses.push_back(pose);
            }
        }

        return path;
    }


    // ------------------------------------------------------
    // FollowPath action handling
    // ------------------------------------------------------

    void try_send_goal()
    {
        if (goal_sent_) {
            return;
        }

        if (!action_client_->action_server_is_ready()) {
            RCLCPP_INFO(
                this->get_logger(),
                "Action server not available yet: %s",
                action_name_.c_str()
            );

            return;
        }

        goal_sent_ = true;

        server_timer_->cancel();

        FollowPath::Goal goal;

        // Use the current ROS clock.
        // If use_sim_time is enabled, this is simulation time.

        const auto stamp = this->get_clock()->now();

        goal.path = path_;

        goal.path.header.frame_id = frame_id_;
        goal.path.header.stamp = stamp;

        for (auto & pose : goal.path.poses) {
            pose.header.frame_id = frame_id_;
            pose.header.stamp = stamp;
        }

        goal.controller_id = controller_id_;
        goal.goal_checker_id = goal_checker_id_;
        goal.progress_checker_id = progress_checker_id_;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending path with %zu poses",
            goal.path.poses.size()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "controller_id=\"%s\", goal_checker_id=\"%s\", "
            "progress_checker_id=\"%s\"",
            goal.controller_id.c_str(),
            goal.goal_checker_id.c_str(),
            goal.progress_checker_id.c_str()
        );

        // Publish the same path that will be sent to Nav2.

        path_.header.stamp = stamp;

        for (auto & pose : path_.poses) {
            pose.header.stamp = stamp;
        }

        path_publisher_->publish(path_);

        // Configure asynchronous action callbacks.

        rclcpp_action::Client<FollowPath>::SendGoalOptions
            send_goal_options;

        send_goal_options.goal_response_callback =
            std::bind(
                &PathPublisher::goal_response_callback,
                this,
                std::placeholders::_1
            );

        send_goal_options.feedback_callback =
            std::bind(
                &PathPublisher::feedback_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            );

        // Send the goal without blocking the executor.

        action_client_->async_send_goal(
            goal,
            send_goal_options
        );
    }


    void goal_response_callback(
        const GoalHandleFollowPath::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Nav2 rejected the FollowPath goal"
            );

            return;
        }

        goal_handle_ = goal_handle;

        RCLCPP_INFO(
            this->get_logger(),
            "FollowPath goal accepted"
        );

        // Request the result asynchronously.

        action_client_->async_get_result(
            goal_handle_,
            std::bind(
                &PathPublisher::result_callback,
                this,
                std::placeholders::_1
            )
        );
    }


    void feedback_callback(
        GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback>
            feedback)
    {
        // Throttle feedback log output to once per second.

        const double now =
            this->get_clock()->now().seconds();

        if (now - last_feedback_log_time_ < 1.0) {
            return;
        }

        last_feedback_log_time_ = now;

        RCLCPP_INFO(
            this->get_logger(),
            "Distance to goal: %.2f m, speed: %.2f m/s",
            feedback->distance_to_goal,
            feedback->speed
        );
    }


    void result_callback(
        const GoalHandleFollowPath::WrappedResult & wrapped_result)
    {
        const auto & result = wrapped_result.result;
        const auto status = wrapped_result.code;

        if (
            status ==
            rclcpp_action::ResultCode::SUCCEEDED
        ) {
            RCLCPP_INFO(
                this->get_logger(),
                "Nav2 completed the path successfully"
            );
        }
        else if (
            status ==
            rclcpp_action::ResultCode::CANCELED
        ) {
            RCLCPP_WARN(
                this->get_logger(),
                "FollowPath goal was canceled"
            );
        }
        else if (
            status ==
            rclcpp_action::ResultCode::ABORTED
        ) {
            RCLCPP_ERROR(
                this->get_logger(),
                "FollowPath failed: action_status=%d, "
                "error_code=%d, error_msg=\"%s\"",
                static_cast<int>(status),
                static_cast<int>(result->error_code),
                result->error_msg.c_str()
            );
        }
        else {
            RCLCPP_ERROR(
                this->get_logger(),
                "FollowPath finished with unknown result status: %d",
                static_cast<int>(status)
            );
        }
    }
};


// ----------------------------------------------------------
// Main
// ----------------------------------------------------------

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node =
            std::make_shared<PathPublisher>();

        rclcpp::spin(node);
    }
    catch (const std::exception & exc) {
        RCLCPP_ERROR(
            rclcpp::get_logger("path_publisher"),
            "Node terminated with an exception: %s",
            exc.what()
        );

        rclcpp::shutdown();

        return 1;
    }

    rclcpp::shutdown();

    return 0;
}