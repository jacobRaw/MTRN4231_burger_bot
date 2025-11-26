#include <memory>
#include <string>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.h"
#include "moveit_msgs/msg/joint_constraint.h"
#include "moveit_msgs/msg/orientation_constraint.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/action/movement.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
 
class MoveitPathPlanningServer
{
public:
  using Movement = custom_interfaces::action::Movement;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;
 
  explicit MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Action Server...");
 
    // Initialize MoveGroupInterface with provided node
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");
 
    // Declare parameters
    node_->declare_parameter("planning_time", 20.0);
    node_->declare_parameter("goal_joint_tolerance", 0.001);
    node_->declare_parameter("goal_position_tolerance", 0.001);
    node_->declare_parameter("goal_orientation_tolerance", 0.001);
    // Apply MoveIt parameters
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
    move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
    move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
 
    move_group_->setPlannerId("TRRTkConfigDefault");
 
    move_group_->setMaxVelocityScalingFactor(0.10);
    move_group_->setMaxAccelerationScalingFactor(0.10);
 
    setupCollisionObjects();
 
    // Initialize Action Server
    action_server_ = rclcpp_action::create_server<Movement>(
      node_,
      "moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveitPathPlanningServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveitPathPlanningServer::handle_accepted, this, std::placeholders::_1)
    );
  }
 
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Movement>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  geometry_msgs::msg::Pose target_pose_;
  double speed_scale_ = 0.1;
 
  moveit_msgs::msg::Constraints constraints_;
 
  const std::string VER_ELBOW = "LEFT_ELBOW";
  const std::string ELBOW = "ELBOW";
  const std::string ORIEN = "ORIEN";
  const std::string WRIST_1 = "WRIST1";
  const std::string FULL = "FULL";
 
  // ======== Action Server Callbacks ========
 
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Movement::Goal> goal)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal request: %s", goal->command.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
 
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Received cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    std::thread{std::bind(&MoveitPathPlanningServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }
 
  // ======== Execution Logic ========
 
  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Movement::Feedback>();
    auto result = std::make_shared<Movement::Result>();
 
    RCLCPP_INFO(node_->get_logger(), "Executing goal: %s", goal->command.c_str());
    feedback->status = "Processing target...";
    goal_handle->publish_feedback(feedback);
 
    if (goal->positions.size() != 6) {
      RCLCPP_INFO(node_->get_logger(), "Expected 6 position elements, got %zu", goal->positions.size());
      result->success = false;
      goal_handle->abort(result);
      return;
    }
 
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
 
    bool target_set = false;
    if (goal->command == "cartesian") {
      target_set = set_cartesian_target(goal->positions);
    } else if (goal->command == "joint") {
      target_set = set_joint_target(goal->positions);
    }
    else {
      RCLCPP_INFO(node_->get_logger(), "Unknown movement command.");
      return;
 
    }
 
    if (!target_set) {
      RCLCPP_INFO(node_->get_logger(), "Failed to set target.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
 
    if (goal->constraints_identifier != "NONE") {
      move_group_->setPathConstraints(set_constraint(goal->constraints_identifier));
    }
 
    feedback->status = "Planning and executing...";
    goal_handle->publish_feedback(feedback);
 
    bool success = false;
 
    if (goal->command == "cartesian") {
      success = plan_and_execute();
    }
    else if (goal->command == "joint") {
      success = plan_and_execute_joint();
    }
 
    constraints_.joint_constraints.clear();
    constraints_.position_constraints.clear();
    constraints_.orientation_constraints.clear();
    constraints_.visibility_constraints.clear();
    constraints_.name.clear();
 
 
 
    if (success) {
      feedback->status = "Execution complete.";
      goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(node_->get_logger(), "Success from cartesian.");
      result->success = true;
      goal_handle->succeed(result);
    } else {
      feedback->status = "Execution failed.";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->abort(result);
    }
  }
 
  // ======== Helper Functions ========
 
  bool set_cartesian_target(const std::vector<double>& positions)
  {
    tf2::Quaternion q;
    q.setRPY(positions[3], positions[4], positions[5]);
    q.normalize();
 
    target_pose_.position.x = positions[0];
    target_pose_.position.y = positions[1];
    target_pose_.position.z = positions[2];
    target_pose_.orientation = tf2::toMsg(q);
    return true;
  }
 
  bool set_joint_target(const std::vector<double>& positions)
  {
    std::map<std::string, double> joint_targets = {
      {"shoulder_pan_joint", positions[5]},
      {"shoulder_lift_joint", positions[0]},
      {"elbow_joint", positions[1]},
      {"wrist_1_joint", positions[2]},
      {"wrist_2_joint", positions[3]},
      {"wrist_3_joint", positions[4]}
    };
    move_group_->setJointValueTarget(joint_targets);
    return true;
  }
 
  moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str)
  {
    moveit_msgs::msg::Constraints constraints;
 
    //# the bound to be achieved is [position - tolerance_below, position + tolerance_above]
 
    if (str_contains(constraint_str, WRIST_1)) {
      moveit_msgs::msg::JointConstraint wrist_constraint;
      wrist_constraint.joint_name = "wrist_1_joint";
      const double min_angle = -180.0 * M_PI / 180.0;
      const double max_angle = 0 * M_PI / 180.0;
      const double mid_angle = (min_angle + max_angle) / 2.0;
      wrist_constraint.position = mid_angle;
      wrist_constraint.tolerance_below = mid_angle - min_angle;
      wrist_constraint.tolerance_above = max_angle - mid_angle;
      wrist_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(wrist_constraint);
    }
 
    if (str_contains(constraint_str, ELBOW) || str_contains(constraint_str, FULL)) {
      moveit_msgs::msg::JointConstraint elbow_constraint;
      elbow_constraint.joint_name = "elbow_joint";
      const double min_angle = 20.0 * M_PI / 180.0;
      const double max_angle = 150.0 * M_PI / 180.0;
      const double mid_angle = (min_angle + max_angle) / 2.0;
      elbow_constraint.position = mid_angle;
      elbow_constraint.tolerance_below = mid_angle - min_angle;
      elbow_constraint.tolerance_above = max_angle - mid_angle;
      elbow_constraint.weight = 1.0;
      RCLCPP_INFO(node_->get_logger(), "Elbow constraint applied.");
      constraints.joint_constraints.push_back(elbow_constraint);
    }
 
    constraints_ = constraints;
 
    return constraints;
  }
 
  bool plan_and_execute()
  {
    move_group_->setStartStateToCurrentState();
 
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(target_pose_);
 
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, constraints_, true);
 
    if (fraction < 0.90) {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian Path failed: %.2f", fraction);
      return false;
    }
 
    // for (auto &point : trajectory.joint_trajectory.points) {
    //   for (auto &v : point.velocities) v *= speed_scale_;
    //   for (auto &a : point.accelerations) a *= speed_scale_;
    // }
 
    moveit_msgs::msg::RobotTrajectory trajectory_slow;
    // add timing Note: you have to convert it to a RobotTrajectory Object (not message) and back
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    robot_trajectory::RobotTrajectory r_trajec(move_group_->getRobotModel(), move_group_->getName());
    r_trajec.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
    iptp.computeTimeStamps(r_trajec, 1, 1);
    r_trajec.getRobotTrajectoryMsg(trajectory);
    iptp.computeTimeStamps(r_trajec, speed_scale_, speed_scale_);
    r_trajec.getRobotTrajectoryMsg(trajectory_slow);
 
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_slow;
    move_group_->execute(plan);
    return true;
  }
 
    bool plan_and_execute_joint()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = 3;
    
    while (!success && attempts < max_attempts) {
        success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        attempts++;
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);
            move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
        }
    }
    move_group_->setPlanningTime(20.0);
 
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Plan successful after %d attempts. Executing...", attempts);
        move_group_->execute(plan);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed after %d attempts.", max_attempts);
        return false;
    }
  }
 
 
  void setupCollisionObjects()
  {
    std::string frame_id = "world";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, -0.01, 0.85, 0.25, 0.013, frame_id, "table"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.2, frame_id, "ceiling"));
  }
 
  moveit_msgs::msg::CollisionObject generateCollisionObject(float sx, float sy, float sz, float x, float y, float z,
                                                            const std::string& frame_id, const std::string& id)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {sx, sy, sz};
 
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
 
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }
 
  bool str_contains(const std::string& str, const std::string& substr)
  {
    if (substr.empty()) return false;
    return str.find(substr) != std::string::npos;
  }
};
 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
 
  // ✅ Create a shared node and pass it to the server
  auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_action_server_node");
  auto server = std::make_shared<MoveitPathPlanningServer>(node);
 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
 
 
 
// (this move to home first)
// ros2 action send_goal /moveit_path_plan custom_interfaces/action/Movement "{command: 'joint', positions: [-1.57079633, 0.767945, -0.767945, -1.57079633, 0.0, 0.0], constraints_identifier: 'NONE'}"
 
// (to pickup pose down)
// ros2 action send_goal /moveit_path_plan custom_interfaces/action/Movement "{command: 'cartesian', positions: [0.4, 0.1, 0.3, 3.1415926536, 0.0, -1.5707963268], constraints_identifier: 'FULL'}"

// this command will send arm with end effector attached to the bolt "3"
// ros2 action send_goal /moveit_path_plan custom_interfaces/action/Movement "{command: 'cartesian', positions: [0.23, 0.519, 0.194, 3.1415926536, 0.0, -1.7507963268], constraints_identifier: 'FULL'}"
