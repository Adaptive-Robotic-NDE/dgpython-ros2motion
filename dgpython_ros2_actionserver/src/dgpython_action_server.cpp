#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <limits>

#include <cstdio>

#include "dgpython_ros2_actioninterface/action/dg_python.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "dgpython_ros2_actionserver/visibility_control.h"

namespace dgpython_ros2_actionserver
{
class DgPythonActionServer : public rclcpp::Node
{
public:
  using DgPython = dgpython_ros2_actioninterface::action::DgPython;
  using GoalHandleDgPython = rclcpp_action::ServerGoalHandle<DgPython>;

  DGPYTHON_ROS2_ACTIONSERVER_PUBLIC
  explicit DgPythonActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dgpython_action_server", options),
    move_group_interface_(nullptr)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<DgPython>(
      this,
      "dgpython",
      std::bind(&DgPythonActionServer::handle_goal, this, _1, _2),
      std::bind(&DgPythonActionServer::handle_cancel, this, _1),
      std::bind(&DgPythonActionServer::handle_accepted, this, _1));

     }

private:
  rclcpp_action::Server<DgPython>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DgPython::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with destination (%f, %f, %f)", goal->destination.position.x, goal->destination.position.y, goal->destination.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDgPython> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDgPython> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DgPythonActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDgPython> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Preparing to execute goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DgPython::Result>();

    if (!this->move_group_interface_) {
	this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
	this->move_group_interface_->setGoalTolerance(0.00005);
	RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", this->move_group_interface_->getPlanningFrame().c_str());
	RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", this->move_group_interface_->getEndEffectorLink().c_str());
    }

    auto const target_pose = [](geometry_msgs::msg::Pose msg){
  	    			geometry_msgs::msg::Pose out;
	 			out.orientation.w = msg.orientation.w;
				out.orientation.x = msg.orientation.x;
				out.orientation.y = msg.orientation.y;
				out.orientation.z = msg.orientation.z;
				out.position.x = msg.position.x;
				out.position.y = msg.position.y;
				out.position.z = msg.position.z;
				return out;
				}(goal->destination);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = this->move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (goal_handle->is_canceling()) {
        //result->sequence = sequence;
        result->success = false;
        goal_handle->canceled(result);
	this->move_group_interface_->stop();
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    moveit::core::MoveItErrorCode status;
    if(fraction > 0 && (fraction - 1.0) <= std::numeric_limits<double>::epsilon() && rclcpp::ok()) {
			status = this->move_group_interface_->execute(trajectory);
		}
		else {
 			this->move_group_interface_->stop();
		        result->success = false;
			goal_handle->abort(result);
			RCLCPP_ERROR(this->get_logger(), "Planning failed");
      return;      
		}   

    // Check if goal is done
    if (rclcpp::ok() && status.val == moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      return;
    }
    else {
      this->move_group_interface_->stop();
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Failed to reach goal, error code %d", status.val);
      return;
    }
  }
};  // class DgPythonActionServer

}  // namespace dgpython_ros2_actionserver

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<dgpython_ros2_actionserver::DgPythonActionServer>());
	rclcpp::shutdown();	
	return 0;
}
