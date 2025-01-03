#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_pkg/action/reach.hpp"

class ReachActionServer : public rclcpp::Node, public std::enable_shared_from_this<ReachActionServer>{
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
        // Defer move_group_ initialization  
        std::cout << "Initiated!" << std::endl; 

    }

    void initialize(std::shared_ptr<ReachActionServer> node) {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, 
        "panda_arm"
    );
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }


private:
    rclcpp_action::Server<Reach>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Reach::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request for position x %f, y %f, z %f", goal->goal_position_x,goal->goal_position_y,goal->goal_position_z);
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

        const auto goal = goal_handle->get_goal();

        // Get current pose of the end effector (PoseStamped)
        geometry_msgs::msg::PoseStamped current_pose_stamped = move_group_->getCurrentPose();

        // Extract the Pose from PoseStamped
        geometry_msgs::msg::Pose current_pose = current_pose_stamped.pose;


        // Print the Cartesian position of the end effector
        RCLCPP_INFO(this->get_logger(), "End effector position: [x: %f, y: %f, z: %f]",
                    current_pose.position.x, current_pose.position.y, current_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "End effector orientation: [x: %f, y: %f, z: %f, w: %f]",
                    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);        // Define target pose
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = goal->goal_position_x; // X position from goal
        target_pose.position.y = goal->goal_position_y;                // Example Y position
        target_pose.position.z = goal->goal_position_z;                // Example Z position
        target_pose.orientation.x = current_pose.orientation.x;             // Example orientation
        target_pose.orientation.y = current_pose.orientation.y;             // Example orientation
        target_pose.orientation.z = current_pose.orientation.z;             // Example orientation
        target_pose.orientation.w = current_pose.orientation.w;             // Example orientation

        // Set the target pose in MoveGroupInterface
        move_group_->setPoseTarget(target_pose);
                
        auto moveit_result = move_group_->move();
        auto result = std::make_shared<Reach::Result>();

        if (moveit_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Motion succeeded");
            result->status = "Goal reached";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Motion failed");
            result->status = "Motion failed";
            goal_handle->abort(result);
        }
        
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachActionServer>();
    node->initialize(node); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
