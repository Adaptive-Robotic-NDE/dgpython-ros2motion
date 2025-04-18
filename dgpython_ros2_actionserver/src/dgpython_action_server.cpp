/*#######################################################################
# AdaptiveNDE Project                                                   #
# AFRL-ROS Package                                                      # 
#########################################################################
# Copyright 2023-2024 University of Dayton                              #
#                                                                       #
# Redistribution and use in source and binary forms, with or without    # 
# modification, are permitted provided that the following conditions    #
# are met:                                                              #
#                                                                       # 
# 1. Redistributions of source code must retain the above copyright     #
#    notice, this list of conditions and the following disclaimer.      #
# 2. Redistributions in binary form must reproduce the above copyright  #
#    notice, this list of conditions and the following disclaimer in    #
#    the documentation and/or other materials provided with the         #
#    distribution.                                                      #
# 3. Neither the name of the copyright holder nor the names of its      #
#    contributors may be used to endorse or promote products derived    #
#    from this software without specific prior written permission.      #
#                                                                       #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
# “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR #
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,#
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      #
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, #
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  #
#########################################################################
# This material is based in part upon work supported by the U.S. Air    #
# Force under Contract FA8650-19-D-5230.                                #
#                                                                       #
# DISTRIBUTION A. Approved for public release: distribution unlimited.  #
#########################################################################
# File:  dgpython_action_server.cpp                                                     
# Authors: Tyler Lesthaeghe
# Created: 2023-06-22                                                  
# Description:
#   Dataguzzler/MoveIt Bridge Implementation
# Derived From:                                                         
#   https://moveit.picknik.ai/main/index.html    
#     under BSD 3-Clause License
#########################################################################
# Revision Log                                                          
# 
# 2023-06-22:  Initial Commit
# 2023-11-06:  Added Stop on Failure
# 2023-11-14:  Debugging Statements
# 
#########################################################################
# Development Notes
#
#
#######################################################################*/

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
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/collision_object.hpp>

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

    mesh_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "industrial_reconstruction_mesh", 10, std::bind(&DgPythonActionServer::mesh_callback, this, _1));

//    octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//      "/octomap_binary", 10, std::bind(&DgPythonActionServer::octomap_callback, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<DgPython>(
      this,
      "dgpython",
      std::bind(&DgPythonActionServer::handle_goal, this, _1, _2),
      std::bind(&DgPythonActionServer::handle_cancel, this, _1),
      std::bind(&DgPythonActionServer::handle_accepted, this, _1));

     }

private:
  rclcpp_action::Server<DgPython>::SharedPtr action_server_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr mesh_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  shape_msgs::msg::Mesh collision_mesh_;
  planning_scene::PlanningScenePtr planning_scene_;
  int counter = 1;

  void mesh_callback(const visualization_msgs::msg::Marker& mesh)
  {

    RCLCPP_INFO(this->get_logger(), "Received idustrial reconstruction mesh %d", counter);

    // Convert points from marker to mesh vertices
    collision_mesh_.vertices.clear();
    for (const auto& point : mesh.points) {
        geometry_msgs::msg::Point vertex;
        vertex = point;  // Direct assignment of points
        collision_mesh_.vertices.push_back(vertex);
    }

    // Create triangles from points (assuming points are arranged in triangles)
    collision_mesh_.triangles.clear();
    for (size_t i = 0; i < mesh.points.size(); i += 3) {
        shape_msgs::msg::MeshTriangle triangle;
        triangle.vertex_indices[0] = i;
        triangle.vertex_indices[1] = i + 1;
        triangle.vertex_indices[2] = i + 2;
        collision_mesh_.triangles.push_back(triangle);
    }

    collision_object.header.frame_id = "world";
    collision_object.id = "object_" + std::to_string(counter);

    collision_object.meshes.push_back(collision_mesh_);
    collision_object.mesh_poses.push_back(mesh.pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
    counter++;
    
  }
//  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
//  {
//    RCLCPP_INFO(this->get_logger(), "Received Octomap");
//  
//    // Convert the octomap message to an octree
//    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
//  
//    if (octree)
//    {
//      // Convert the octree to an OctomapWithPose message
//      octomap_msgs::msg::OctomapWithPose octomap_with_pose_msg;
//      octomap_msgs::fullMapToMsg(*octree, octomap_with_pose_msg.octomap);
//      octomap_with_pose_msg.header.frame_id = "world"; // Or the appropriate frame
//      octomap_with_pose_msg.pose.orientation.w = 1.0; // Set default orientation
//  
//      // Get the planning scene
//      if (!planning_scene_) {
//          planning_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_interface_->getRobotModel());
//      }
//  
//      // Process the octomap message
//      planning_scene_->processOctomapMsg(octomap_with_pose_msg.octomap);
//  
//      // Notify the planning scene that the world has changed
//      // planning_scene_->setWorldChanged(true); // No longer needed
//  
//      RCLCPP_INFO(this->get_logger(), "Octomap integrated into planning scene");
//    }
//    else
//    {
//      RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree");
//    }
//  }
//  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
//  {
//    RCLCPP_INFO(this->get_logger(), "Received Octomap");
//  
//    // Convert the octomap message to an octree
//    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
//  
//    if (octree)
//    {
//      // Get the planning scene
//      planning_scene_ = move_group_interface_->getPlanningScene();
//  
//      // Update the planning scene with the octree
//      planning_scene->processOctomap(*octree);
//  
//      // Notify the planning scene that the world has changed
//      planning_scene->setWorldChanged(true);
//  
//      RCLCPP_INFO(this->get_logger(), "Octomap integrated into planning scene");
//    }
//    else
//    {
//      RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree");
//    }
//  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DgPython::Goal> goal)
  {
    if(!goal->state.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with state (%s)", goal->state.c_str());
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else if(!goal->pose_array.poses.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request received pose array with %d poses", goal->pose_array.poses.size());
    }
    else if(goal->destination.position.x != 0.0 && goal->destination.position.y != 0.0 && goal->destination.position.z != 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with destination (%f, %f, %f)", goal->destination.position.x, goal->destination.position.y, goal->destination.position.z);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with no goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
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
    move_group_interface_->setMaxVelocityScalingFactor(goal->velocity);
    move_group_interface_->setMaxAccelerationScalingFactor(goal->acceleration);
    move_group_interface_->setPlanningTime(10.0);
    if(!goal->state.empty()){

        // Retrieve the robot model
      //  const moveit::core::RobotModelPtr robot_model = move_group_interface_.getModel();
      //  if (!robot_model) {
      //      RCLCPP_ERROR(this->get_logger(), "Failed to get robot model.");
      //      return;
      //  }

        // Get the named targets from the SRDF
      //  std::vector<std::string> named_targets;
      //  robot_model->getJointModelGroup("ur_manipulator")->getVariableNames(named_targets);
      //
      //  RCLCPP_INFO(this->get_logger(), "Available named targets:");
      //  for (const auto& target : named_targets) {
      //      RCLCPP_INFO(this->get_logger(), "%s", target.c_str());
      //  }
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::string target_name = goal->state;
        move_group_interface_->setNamedTarget(target_name);
        bool success = (move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_interface_->execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed for named target: %s", target_name.c_str());
        }

        if (rclcpp::ok()) {
            // feedback->progress = 1.0;
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal executed successfully");
            return;
        }
    }
    else if(!goal->pose_array.poses.empty()){

      std::vector<geometry_msgs::msg::Pose> waypoints;
      for (const auto& pose : goal->pose_array.poses) {
            waypoints.push_back(pose);
        }
      RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", waypoints.size());
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
    else if(goal->destination.position.x != 0.0 && goal->destination.position.y != 0.0 && goal->destination.position.z != 0.0)
    {
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
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid goal");
      result->success = false;
      goal_handle->abort(result);
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
