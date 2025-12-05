/**
 * @file brainNode.cpp
 * @author Jacob Rawung
 * @brief Brain node that contains the state machine to process orders from the input node and coordinate
 *       other system components like the robot arm and camera to assemble the burger.
 * 
 * @launch ros2 launch brain input.launch.py
 */

#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <unordered_set>
#include <stdexcept>
#include <limits>
#include <cmath>

#include "custom_interfaces/action/order_request.hpp"
#include "custom_interfaces/msg/ingredients.hpp"
#include "custom_interfaces/action/movement.hpp"
#include "custom_interfaces/srv/gripper_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

enum class State {
  startState,
  pickPlaceState
};

enum class Ingredients {
  topBun,
  bottomBun,
  lettuce,
  tomato,
  cheese,
  patty,
  pickles,
  unknown
};

struct IngredientPos {
  Ingredients ingredient;
  std::vector<double> position; // [0]=x [1]=y [2]=z
};


class BrainNode : public rclcpp::Node
{
public:

  // user_input action server
  using action_interface = custom_interfaces::action::OrderRequest;
  using GoalHandleOrderRequest = rclcpp_action::ServerGoalHandle<action_interface>;

  // path planner action client
  using moveit_action_interface = custom_interfaces::action::Movement;
  using GoalHandleMoveIt = rclcpp_action::ClientGoalHandle<moveit_action_interface>;

  explicit BrainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("brainNode", options)
  {
    RCLCPP_INFO(this->get_logger(), "BrainNode has started.");
	this->declare_parameter<bool>("use_fake_hardware", true);
    this->get_parameter("use_fake_hardware", use_fake_hardware_);

    using namespace std::placeholders;

	RCLCPP_INFO(this->get_logger(), "Use fake hardware: %s", use_fake_hardware_ ? "true" : "false");
    // service call the arduino to control the gripper
    this->gripper_client_ = this->create_client<custom_interfaces::srv::GripperServer>("gripper_server");
    while (rclcpp::ok() && !use_fake_hardware_ && !gripper_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Waiting on Arduino Server to become available...");
    }

    // Create action server to communicate with user input node
    this->action_server_ = rclcpp_action::create_server<action_interface>(
      this,
      "user_interface",
      std::bind(&BrainNode::handle_goal_request, this, _1, _2),
      std::bind(&BrainNode::handle_cancel, this, _1),
      std::bind(&BrainNode::handle_accepted, this, _1)
    );

    // Create action server to communicate with moveIt node
    this->moveit_action_client_ = rclcpp_action::create_client<moveit_action_interface>(
      this,
      "moveit_path_plan"
    );

    send_goal_options = rclcpp_action::Client<moveit_action_interface>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&BrainNode::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&BrainNode::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&BrainNode::result_callback, this, _1);

    // subscribe to perception topic
    this->perception_subscriber_ = this->create_subscription<custom_interfaces::msg::Ingredients>(
      "/ingredients",
      10,
      std::bind(&BrainNode::perception_callback, this, _1)
    );

    // publish collision objects to MoveIt
    this->collision_object_publisher_ = this->create_publisher<custom_interfaces::msg::Ingredients>(
      "/collision_objects",
      10
    );

    // Initialize class attributes
    currentState = State::startState;
  }

private:
	// ROS communication objects
	// ROS paramter to set whether fakehardware is used so we can skip gripper commands
	bool use_fake_hardware_;
	// action server for the input node
	rclcpp_action::Server<action_interface>::SharedPtr action_server_;
	std::shared_ptr<GoalHandleOrderRequest> active_goal_;

	// action client for moveit node
	rclcpp_action::Client<moveit_action_interface>::SharedPtr moveit_action_client_;
	rclcpp_action::Client<moveit_action_interface>::SendGoalOptions send_goal_options;
	rclcpp_action::ResultCode moveit_result_code_;

	// subscription to perception node
	rclcpp::Subscription<custom_interfaces::msg::Ingredients>::SharedPtr perception_subscriber_;

	// service client to control gripper
	rclcpp::Client<custom_interfaces::srv::GripperServer>::SharedPtr gripper_client_;

	// publisher for collision objects
	rclcpp::Publisher<custom_interfaces::msg::Ingredients>::SharedPtr collision_object_publisher_;

	//logic variables
	State currentState;
	std::vector<Ingredients> order_ingredients;
	int ing_index = 0;
	bool terminate = false;

	// shared variables
	bool accept_cv = false;
	bool arm_moving = false;
	std::vector<IngredientPos> available_ingredients;

	// threading variables
	std::mutex mtx;

	// constants
	static constexpr double PI = 3.14159265358979323846;
	static constexpr double HOVER_HEIGHT = 0.3;
	const std::vector<double> HOME_POS = {0.15, 0.490, HOVER_HEIGHT, PI, 0.0, -PI/2.0};
	static constexpr double PICK_HEIGHT = 0.194;
	static constexpr int GRIPPER_WAIT_TIME = 1000;
	const std::string GRIPPER_CLOSE_CMD = "c";
	const std::string GRIPPER_OPEN_CMD = "o";
	
	/********************MAIN STATE LOOP********************/
	/**
	 * @brief Main loop to handle state transitions
	 */
	void run()
	{
		set_order_ingredients();
		auto feedback = std::make_shared<action_interface::Feedback>();
		auto result = std::make_shared<action_interface::Result>();

		rclcpp::Rate loop_rate(5);
		this->input_feedback("Starting order...");
		// Main loop to handle state transitions
		while (rclcpp::ok() && !this->terminate) {
			switch (this->currentState) {
				case State::startState:
					startState();
					break;
				case State::pickPlaceState:
					pickPlaceState();
					break;
			}
			loop_rate.sleep();
		}

		this->report_success();
	}

	/********************STATE FUNCTIONS********************/
	/**
	 * @brief Only runs at start, moves arm to home position and determines all available ingredients
	 */
	void startState() {
		// move arm to home position
		try {
			send_arm_dest(HOME_POS);
		} catch (const std::runtime_error & e) {
			RCLCPP_ERROR(this->get_logger(), "Unable to reach home position: %s", e.what());
		}
		
		// check if the burger is complete
		if (ing_index >= static_cast<int>(order_ingredients.size())){
			this->input_feedback("Order Complete!");
			this->terminate = true;
			return;
		}
		// get information from perception to determine all available ingredients and their positions
		this->set_accept_cv(true);
			//wait until perception is complete
			RCLCPP_INFO(this->get_logger(), "Waiting for perception data...");
			while (rclcpp::ok() && this->get_accept_cv()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // small sleep
		}
		
		this->currentState = State::pickPlaceState;
	}

	/**
	 * @brief moves the arm and picks up the ingredient and moves the ingredient to the stack
	 */
	void pickPlaceState()
	{
		// get the ingredient position for the current index (closest available ingredient) and publish other ingredients as collision objects
		Ingredients target_ingredient = order_ingredients[ing_index];
		std::vector<double> target_position;
		double distance = std::numeric_limits<double>::max();
		auto msg = custom_interfaces::msg::Ingredients();
		for (const auto & ingredientPos : this->get_available_ingredients()) {
			if (ingredientPos.ingredient == target_ingredient && calc_distance(ingredientPos.position[0], ingredientPos.position[1]) < distance) {
				if (!target_position.empty()) {
					msg.ingredients.push_back(create_ingredient_msg(target_ingredient, target_position));
				}
				target_position = ingredientPos.position;
				continue;
			} else {
				msg.ingredients.push_back(create_ingredient_msg(ingredientPos.ingredient, ingredientPos.position));
			}
		}
		this->collision_object_publisher_->publish(msg);

		try {
			// move arm only in the XY plane to the ingredient pos based off the current ingredient index
			send_arm_dest({target_position[0], target_position[1], HOVER_HEIGHT, PI, 0.0, -PI/2.0});
		
			// lower the arm such that the pick has pressure
			double z_rot = -PI/2.0;
			double num_step = 10;
			double orien = 0.0;
			bool found_solution = false;
			double max_rot = (170 * PI) / 180.0;
			for (double i = z_rot; i <= max_rot + z_rot; i += (max_rot/num_step)) {
				// rotate first
				send_arm_dest({target_position[0], target_position[1], HOVER_HEIGHT, PI, 0.0, i});
				// then try moving down
				auto result = send_arm_dest({target_position[0], target_position[1], PICK_HEIGHT, PI, 0.0, i}, true);
				if (result) {
					orien = i;
					found_solution = true;
					break;
				}
			}
			// send_arm_dest({target_position[0], target_position[1], PICK_HEIGHT, PI, 0.0, -PI/2.0});
			if (!found_solution) {
				this->currentState = State::startState;
				throw std::runtime_error("No valid arm orientation found for picking up ingredient.");
			}

			// close the gripper
			send_gripper_command(GRIPPER_CLOSE_CMD);
		
			// raise the arm to hover height
			send_arm_dest({target_position[0], target_position[1], HOVER_HEIGHT, PI, 0.0, orien});
		
			// move arm to the home position
			send_arm_dest(HOME_POS);
		
			// lower arm to the current stack height
			double drop_height = get_drop_height();
			send_arm_dest({HOME_POS[0], HOME_POS[1], drop_height, PI, 0.0, -PI/2.0});
		
			// open the gripper
			send_gripper_command(GRIPPER_OPEN_CMD);
		
			// increment the ingredient index
			ing_index++;
			// change to the start state
			this->currentState = State::startState;
		} catch (const std::runtime_error & e) {
			if (this->currentState != State::startState) {
				this->terminate = true;
				RCLCPP_ERROR(this->get_logger(), "Terminating brain node due to unknown error: %s", e.what());
			}
		return;
		}
	}

	/********************PERCEPTION CALLBACKS ********************************* */
	/**
	 * @brief Callback function to process perceived ingredients
	 * @param msg The message containing the list of perceived ingredients
	 */
	void perception_callback(const custom_interfaces::msg::Ingredients::SharedPtr msg)
	{
		if (!this->get_accept_cv()) {
			return;
		}

		this->clear_available_ingredients();
		// Process the perceived ingredients and update internal state
		for (const auto & ingredient_msg : msg->ingredients) {
			Ingredients ingredient = string_to_ingredient(ingredient_msg.ingredient);
			IngredientPos ingredientPos;
			ingredientPos.ingredient = ingredient;
			ingredientPos.position = std::vector<double>();
			ingredientPos.position = {
				ingredient_msg.pos[0], //x
				ingredient_msg.pos[1], //y
				ingredient_msg.pos[2]  //z
			};
			// Update internal state with perceived ingredient
			this->add_available_ingredient(ingredientPos);
		}

		bool found = this->findIngredient(order_ingredients[ing_index], this->get_available_ingredients());
		if (found) {
			this->set_accept_cv(false);
		} else {
			this->input_feedback("Missing ingredient: " + this->ingredient_to_string(order_ingredients[ing_index]));
		}
	}

	/********************INPUT ACTION SERVER HANDLERS************************ */

	/**
	 * @brief Callback function when goal request is sent to action server
	 * @param uuid The unique identifier for the goal
	 * @param goal The goal message containing the ingredients list
	 * @returns GoalResponse indicating whether to accept or reject the goal
	 */
	rclcpp_action::GoalResponse handle_goal_request(
		const rclcpp_action::GoalUUID &,
		std::shared_ptr<const action_interface::Goal>)
	{
		RCLCPP_INFO(this->get_logger(), "Received ingredients list");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 * @brief Callback function when a cancel request is received from the action client
	 * @param goal_handle The goal handle for the goal to be canceled
	 * @returns CancelResponse indicating whether to accept or reject the cancel request
	 */
	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleOrderRequest>)
	{
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	/**
	 * @brief Callback function when a goal is accepted by the action server
	 * @param goal_handle The goal handle for the accepted goal
	 */
	void handle_accepted(const std::shared_ptr<GoalHandleOrderRequest> goal_handle)
	{
		this->set_active_goal(goal_handle);
		using namespace std::placeholders;
		RCLCPP_INFO(this->get_logger(), "New order received, starting new thread");
		this->terminate = false;
		this->ing_index = 0;
		this->currentState = State::startState;
		this->available_ingredients.clear();
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{&BrainNode::run, this}.detach();
	}

	/**
	 * @brief sends feedback to the input node
	 */
	void input_feedback(const std::string & status) {
		std::lock_guard<std::mutex> lock(mtx);
		auto feedback = std::make_shared<action_interface::Feedback>();
		feedback->status = status;
		auto goal_handle = this->active_goal_;
		goal_handle->publish_feedback(feedback);
	}

	/**
	 * @brief sends result to the user upon successfully completing burger
	 */
	void report_success() {
		std::lock_guard<std::mutex> lock(mtx);
		auto goal_handle = this->active_goal_;
		auto result = std::make_shared<action_interface::Result>();
		result->success = true;
		goal_handle->succeed(result);
	}

	/******************************* MoveIt Action Client Callbacks *************************** */
	
	/**
	 * @brief response after requesting the goal position
	 */
	void goal_response_callback(const GoalHandleMoveIt::SharedPtr & goal_handle)
	{
		if (!goal_handle) {
		RCLCPP_ERROR(this->get_logger(), "Goal was rejected by movit_server");
		this->set_arm_moving(false);
		} else {
		RCLCPP_INFO(this->get_logger(), "Goal accepted by moveit_server, waiting for result");
		}
	}

	/**
	 * @brief prints feedback from the moveit action server
	 */
	void feedback_callback(
		GoalHandleMoveIt::SharedPtr,
		const std::shared_ptr<const custom_interfaces::action::Movement::Feedback> feedback)
	{
		RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
	}

	/**
	 * @brief Callback function when result is received from action server
	 * @param result The result from the action server
	 */
	void result_callback(const rclcpp_action::ClientGoalHandle<custom_interfaces::action::Movement>::WrappedResult & result)
	{
		this->moveit_result_code_ = result.code;
		this->set_arm_moving(false);
	}

	/**
	 * @brief send the x, y, z, roll, pitch and yaw position to move the robot arm
	 */
	bool send_arm_dest(std::vector<double> pose, bool picking_up = false) {
		if (this->terminate) {
			return false;
		}

		if (!this->moveit_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
			this->terminate = true;
			return false;
		}

		auto goal_msg = moveit_action_interface::Goal();
		goal_msg.command = "cartesian";
		goal_msg.constraints_identifier = "FULL";
		goal_msg.positions = pose;

		this->moveit_action_client_->async_send_goal(goal_msg, send_goal_options);
		this->set_arm_moving(true);
		wait_for_arm();
		if (moveit_result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
			this->input_feedback("Successfully moved to target.");
			return true;
		} else if (!picking_up) {
			this->input_feedback("Path planning failed, waiting for targets to become reachable or obstacles to move.");
			this->currentState = State::startState;
			throw std::runtime_error("MoveIt path planning failed");
		} else {
			return false;
		}
	}

	/**************************GRIPPER SERVER HANDLERS ************************** */
	void gripper_server_response(const rclcpp::Client<custom_interfaces::srv::GripperServer>::SharedFuture future) {
		return;
		auto response = future.get();
		if (!response) {
			RCLCPP_ERROR(this->get_logger(), "Failed to call gripper server");
			return;
		} else if (!response->success) {
			RCLCPP_ERROR(this->get_logger(), "The gripper server reported failure: %s", response->message.c_str());
			return;
		} else {
			RCLCPP_INFO(this->get_logger(), "Gripper command succeeded");
		}
	}

	/************************HELPER FUNCTIONS********************/
	/**
	 * @brief Compare available ingredients with order ingredients
	 * @param orderIngredients (std::vector<Ingredients>)
	 * @param availableIngredients (std::vector<itemPos>)
	 * @returns list of missing ingredients
	 */
	bool findIngredient(Ingredients targetIngredient, std::vector<IngredientPos> availableIngredients)
	{
		bool found = false;
		for (IngredientPos ingredient : availableIngredients) {
		if (targetIngredient == ingredient.ingredient) {
			found = true;
		}
		}

		return found;
	}

	/**
	 * @brief Convert string to Ingredients enum
	 * @param ingredient_str The ingredient string
	 * @returns Ingredients enum
	 */
	Ingredients string_to_ingredient(const std::string & ingredient_str)
	{
		if(ingredient_str == "lettuce") {
		return Ingredients::lettuce;
		} else if(ingredient_str == "tomato") {
		return Ingredients::tomato;
		} else if(ingredient_str == "cheese") {
		return Ingredients::cheese;
		} else if(ingredient_str == "patty") {
		return Ingredients::patty;
		} else if(ingredient_str == "bun_top") {
		return Ingredients::topBun;
		} else if(ingredient_str == "bun_bottom") {
		return Ingredients::bottomBun;
		} else if(ingredient_str == "pickles") {
		return Ingredients::pickles;
		} else {
			RCLCPP_ERROR(this->get_logger(), "Unknown ingredient string: %s", ingredient_str.c_str());
			return Ingredients::unknown;
		}
	}

	/**
	 * @brief Convert Ingredients enum to string
	 * @param ingredient The Ingredients enum
	 * @returns string representation of the ingredient
	 */
	std::string ingredient_to_string(const Ingredients & ingredient)
	{
		switch (ingredient) {
		case Ingredients::lettuce:
			return "lettuce";
		case Ingredients::tomato:
			return "tomato";
		case Ingredients::cheese:
			return "cheese";
		case Ingredients::patty:
			return "patty";
		case Ingredients::topBun:
			return "bun_top";
		case Ingredients::bottomBun:
			return "bun_bottom";
		case Ingredients::pickles:
			return "pickles";
		default:
			return "unknown";
		}
	}

	/**
	 * @brief sets the accept_cv variable to know when to accept new perception data
	 * @param val The value to set accept_cv to
	 */
	void set_accept_cv(bool val) {
		std::lock_guard<std::mutex> lock(mtx);
		this->accept_cv = val;
	}

	/**
	 * @brief gets the accept_cv variable to read when to accept new perception data
	 * @returns accept_cv value
	 */
	bool get_accept_cv() {
		std::lock_guard<std::mutex> lock(mtx);
		return this->accept_cv;
	}

	/**
	 * @brief adds an ingredient to the available_ingredients vector
	 * @param ingredient The ingredient to add
	 */
	void add_available_ingredient(IngredientPos ingredient) {
		std::lock_guard<std::mutex> lock(mtx);
		this->available_ingredients.push_back(ingredient);
	}

	/**
	 * @brief clears the available_ingredients vector
	 */
	void clear_available_ingredients() {
		std::lock_guard<std::mutex> lock(mtx);
		this->available_ingredients.clear();
	}

	/**
	 * @brief gets the available_ingredients vector
	 * @returns available_ingredients vector
	 */  
	std::vector<IngredientPos> get_available_ingredients() {
		std::lock_guard<std::mutex> lock(mtx);
		return this->available_ingredients;
	}

	/**
	 * @brief sets the active goal handle
	 * @param goal The goal handle to set as active
	 */
	void set_active_goal(const std::shared_ptr<GoalHandleOrderRequest> goal) {
		std::lock_guard<std::mutex> lock(mtx);
		this->active_goal_ = goal;
	}

	/**
	 * @brief sets the order ingredients from the active goal
	 */
	void set_order_ingredients() {
		std::lock_guard<std::mutex> lock(mtx);
		auto goal_handle = this->active_goal_;
		order_ingredients.clear();
		const auto goal = goal_handle->get_goal();
		for (const auto & ingredient : goal->ingredients) {
		RCLCPP_INFO(this->get_logger(), "Order ingredient: %s", ingredient.c_str());
		order_ingredients.push_back(string_to_ingredient(ingredient));
		}
	}

	/**
	 * @brief sets the arm_moving variable
	 */
	void set_arm_moving(bool val) {
		std::lock_guard<std::mutex> lock(mtx);
		this->arm_moving = val;
	}

	/**
	 * @brief gets the current state of whether the arm is moving
	 */
	bool get_arm_moving() {
		std::lock_guard<std::mutex> lock(mtx);
		return this->arm_moving;
	}

	double get_drop_height() {
		double drop_height = PICK_HEIGHT;
		if (ing_index == 0) {
		return drop_height;
		}

		for (int i = 0; i <= ing_index; i++) {
		drop_height += ingredient_drop_height(order_ingredients[i]);
		}
		return drop_height;
	}

	double ingredient_drop_height(Ingredients ingredient) {
		switch (ingredient) {
		case Ingredients::bottomBun:
			return 0.02;
		case Ingredients::patty:
			return 0.008;
		case Ingredients::cheese:
			return 0.003;
		case Ingredients::tomato:
			return 0.005;
		case Ingredients::lettuce:
			return 0.001;
		case Ingredients::pickles:
			return 0.003;
		case Ingredients::topBun:
			return 0.02;
		default:
			return 0.0;
		}
	}

	void send_gripper_command(std::string command) {
		auto request = std::make_shared<custom_interfaces::srv::GripperServer::Request>();
		request->command = command;

		auto future_result = gripper_client_->async_send_request(
			request,
			std::bind(&BrainNode::gripper_server_response, this, std::placeholders::_1)
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_WAIT_TIME)); 
	}

	void wait_for_arm() {
		while(rclcpp::ok() && this->get_arm_moving()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); // small sleep
		}
	}

	double calc_distance(double x, double y) {
		return std::sqrt(pow(x - HOME_POS[0], 2) + pow(y - HOME_POS[1], 2));
	}

	custom_interfaces::msg::IngredientPos create_ingredient_msg(Ingredients ingredient, std::vector<double> position) {
		custom_interfaces::msg::IngredientPos ingredient_msg;
		ingredient_msg.ingredient = this->ingredient_to_string(ingredient);
		ingredient_msg.pos = position;
		return ingredient_msg;
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BrainNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
