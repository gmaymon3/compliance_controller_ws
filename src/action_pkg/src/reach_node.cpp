#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_pkg/action/reach.hpp"

class ReachActionServer : public rclcpp::Node {
public:
    using Reach = action_pkg::action::Reach;
    using GoalHandleReach = rclcpp_action::ServerGoalHandle<Reach>;

    ReachActionServer() : Node("reach_node") {
        action_server_ = rclcpp_action::create_server<Reach>(
            this,
            "reach",
            std::bind(&ReachActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ReachActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ReachActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<Reach>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Reach::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request for position %f", goal->goal_position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleReach> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Canceling goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleReach> goal_handle) {
        std::thread{std::bind(&ReachActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleReach> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<Reach::Feedback>();
        auto result = std::make_shared<Reach::Result>();
        feedback->success = true;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Feedback published");

        result->actual_position = goal_handle->get_goal()->goal_position;
        result->status = "Goal reached";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
