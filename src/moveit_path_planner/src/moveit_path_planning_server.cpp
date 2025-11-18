#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/srv/movement_request.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h> 

class MoveitPathPlanningServer {
public:
  MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Server...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_,
      "ur_arm"  
    );
    node_->declare_parameter("planning_time", 25.0);
    setupCollsionObjects();

    service_ = node_->create_service<custom_interfaces::srv::MovementRequest>(
      "/moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2)
    );
  }


moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str) {
		moveit_msgs::msg::Constraints constraints;
	
		if (str_contains(constraint_str, ORIEN)) {
			RCLCPP_INFO(node_->get_logger(), "should not be triggered.");
			constraints.orientation_constraints.push_back(set_orientation_constraint());
		}

		// Joint Constraints

		// Elbow -> no less than 60
		//       -> no more than 150
		// Joint constraint for elbow (50° to 145°)

		if (str_contains(constraint_str, ELBOW)) {
			moveit_msgs::msg::JointConstraint elbow_constraint;
			elbow_constraint.joint_name = "elbow_joint";
			
			// Convert degrees to radians
			const double min_angle = 60.0 * M_PI / 180.0;   // 60° in radians (~1.0472)
			const double max_angle = 150.0 * M_PI / 180.0;  // 150° in radians (~2.6179)
			
			// Calculate midpoint (105° which is between 60° and 150°)
			const double midpoint = (min_angle + max_angle) / 2.0;  // ~1.8326 radians
			
			// Set constraints
			elbow_constraint.position = midpoint;
			elbow_constraint.tolerance_below = 1.0;  // ~0.7854 radians (45°)
			elbow_constraint.tolerance_above = max_angle - midpoint;  // ~0.7854 radians (45°)
			elbow_constraint.weight = 1.0;
			
			RCLCPP_INFO(node_->get_logger(), "elbow constraint implemented.");
			
			constraints.joint_constraints.push_back(elbow_constraint);
		}

		if (str_contains(constraint_str, WRIST_1)) {
			moveit_msgs::msg::JointConstraint wrist_constraint;
			wrist_constraint.joint_name = "wrist_1_joint";
			
			// Convert degrees to radians (-71° to -218°)
			const double min_angle = 218.0 * M_PI / 180.0;  // ≈ -3.8048 radians
			const double mid_angle = 144.5 * M_PI / 180.0;  // Midpoint (-144.5° ≈ -2.5220 rad)
			const double max_angle = 71.0 * M_PI / 180.0;   // ≈ -1.2392 radians
		
			// Configure constraint
			wrist_constraint.position = mid_angle;           // Center of range
			wrist_constraint.tolerance_below = 1.7;
			// wrist_constraint.tolerance_below = 0.01;
			// wrist_constraint.tolerance_above = 0.01;
			wrist_constraint.tolerance_above = 1.2828;
			wrist_constraint.weight = 1.0;
			
			constraints.joint_constraints.push_back(wrist_constraint);
		}

		return constraints;
	}

  // moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str) {
	
  //   moveit_msgs::msg::Constraints constraints;
	//   constraints.orientation_constraints.push_back(set_orientation_constraint());
    
	// 	return constraints;

	// }

  // Setting orientation constraints
  moveit_msgs::msg::OrientationConstraint set_orientation_constraint() {
    moveit_msgs::msg::OrientationConstraint orientation_constraint;

    orientation_constraint.header.frame_id = move_group_->getPlanningFrame();
		orientation_constraint.link_name = move_group_->getEndEffectorLink();
		orientation_constraint.absolute_x_axis_tolerance = 0.001;
		orientation_constraint.absolute_y_axis_tolerance = 0.001;
		orientation_constraint.absolute_z_axis_tolerance = 0.001; //0.001
		orientation_constraint.weight = 1;

		tf2::Quaternion q;
    RCLCPP_INFO(node_->get_logger(), "Setting orientation constraint...");
    // Set orientation facing downwards

    q.setRPY(0, M_PI, 0);
    orientation_constraint.orientation = tf2::toMsg(q);
    RCLCPP_INFO(node_->get_logger(), "Orientation constraint applied.");

		return orientation_constraint;
  }

  // Setting collision objects
  void setupCollsionObjects(){
    RCLCPP_INFO(node_->get_logger(), "Applying collision objects to planning scene...");
    std::string frame_id = move_group_->getPlanningFrame();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Add collision object to the scene
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.45, 0.5, frame_id, "backwall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.45, 0.25, 0.5, frame_id, "sidewall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table"));
    //planning_scene_interface.applyCollisionObject(generateCollisionObject(0, 0, 0, 0, 0, 0, frame_id, "ceiling"));
    RCLCPP_INFO(node_->get_logger(), "Collision objects applied.");

  }
  
  auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, std::string& frame_id, const std::string& id) -> moveit_msgs::msg::CollisionObject {

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;

  }


  void handle_request(
    const std::shared_ptr<custom_interfaces::srv::MovementRequest::Request> request,
    std::shared_ptr<custom_interfaces::srv::MovementRequest::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request. Command: %s", request->command.c_str());

    if (request->positions.size() != 6) {
        RCLCPP_ERROR(node_->get_logger(), "Expected 6 position elements, got %zu", request->positions.size());
        response->success = false;
        return;
    }

    // Clear previous targets and constraints
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    // Handle different command types
    bool target_set = false;
    if (request->command == "cartesian") {
        target_set = set_cartesian_target(request->positions);
    } 
    else if (request->command == "joint") {
        target_set = set_joint_target(request->positions);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid command: %s", request->command.c_str());
        response->success = false;
        return;
    }

    if (!target_set) {
        response->success = false;
        return;

    }

    // Setting Constraint
    if (request->constraints_identifier != NONE) {
        move_group_->setPathConstraints(set_constraint(request->constraints_identifier));
    }

    // Common planning and execution logic
    response->success = plan_and_execute();
}

bool plan_and_execute() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempts = 0;
    const int max_attempts = 100;
    
    while (!success && attempts < max_attempts) {
        success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        attempts++;
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "Planning attempt %d failed, retrying...", attempts);
            move_group_->setPlanningTime(move_group_->getPlanningTime() + 2.0);
        }
    }
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Plan successful after %d attempts. Executing...", attempts);
        move_group_->execute(plan);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed after %d attempts.", max_attempts);
        return false;
    }
  }

  bool str_contains(const std::string& str, const std::string& substr) {
    if (substr.empty()) return false;
    return str.find(substr) != std::string::npos;
  }

private:
  bool set_cartesian_target(const std::vector<double>& positions) {
    try {
        geometry_msgs::msg::Pose target_pose = create_pose_from_positions(positions);
        move_group_->setPoseTarget(target_pose);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set Cartesian target: %s", e.what());
        return false;
    }
  }

  bool set_joint_target(const std::vector<double>& positions) {
    try {
        auto joint_targets = create_joint_map_from_positions(positions);
        move_group_->setJointValueTarget(joint_targets);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set joint target: %s", e.what());
        return false;
    }
  }
  
  geometry_msgs::msg::Pose create_pose_from_positions(const std::vector<double>& positions) {
    
    tf2::Quaternion q;
    q.setRPY(positions[3], positions[4], positions[5]);
    q.normalize();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = positions[0];
    target_pose.position.y = positions[1];
    target_pose.position.z = positions[2];
    target_pose.orientation = tf2::toMsg(q);
    
    return target_pose;
  }

  std::map<std::string, double> create_joint_map_from_positions(const std::vector<double>& positions) {
    return {
        {"shoulder_pan_joint", positions[5]},
        {"shoulder_lift_joint", positions[0]},
        {"elbow_joint", positions[1]},
        {"wrist_1_joint", positions[2]},
        {"wrist_2_joint", positions[3]},
        {"wrist_3_joint", positions[4]}
    };
  }


  const int NO_CONSTRAINT = 0;
  const int ORIENTATION_CONSTRAINT_DOWN = 1;
  const int ORIENTATION_CONSTRAINT_LEFT = 2;
  const std::string DOWN = "DOWN";
  const std::string UP =  "UP";
  const std::string LEFT = "LEFT";
  const std::string RIGHT = "RIGHT";
  const std::string NONE = "NONE";
  const std::string VER_ELBOW = "LEFT_ELBOW";
  const std::string ELBOW = "ELBOW";
  const std::string ORIEN = "ORIEN";
  const std::string WRIST_1= "WRIST1";


  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<custom_interfaces::srv::MovementRequest>::SharedPtr service_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_server");
  MoveitPathPlanningServer server(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

