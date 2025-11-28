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

#include "custom_interfaces/action/order_request.hpp"
#include "custom_interfaces/msg/ingredients.hpp"
#include "custom_interfaces/action/movement.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

enum class State {
  startState,
  homeState,
  stackState,
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

    using namespace std::placeholders;

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

    // Initialize class attributes
    currentState = State::startState;
  }

private:
  // ROS communication objects
  // action server for the input node
  rclcpp_action::Server<action_interface>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleOrderRequest> active_goal_;

  // action client for moveit node
  rclcpp_action::Client<moveit_action_interface>::SharedPtr moveit_action_client_;
  rclcpp_action::Client<moveit_action_interface>::SendGoalOptions send_goal_options;

  // subscription to perception node
  rclcpp::Subscription<custom_interfaces::msg::Ingredients>::SharedPtr perception_subscriber_;
  
  //logic variables
  State currentState;
  std::vector<Ingredients> order_ingredients;

  // shared variables
  bool accept_cv = false;
  std::vector<IngredientPos> available_ingredients;

  // threading variables
  std::mutex mtx;

  // constants
  static constexpr double PI = 3.14159265358979323846;
  const std::vector<double> HOME_POS = {0.23, 0.519, 0.194, PI, 0.0, -PI/2.0};
  
  //arm pose read from the robot arm topic
  
  // available ingredients (std::vector<itemPos>)
  // bin position (itemPos)
  // stack position (itemPos)
  // pickupZ (double)
  // double dropHeight = 0.5;

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
    while (rclcpp::ok()) {
        switch (this->currentState) {
            case State::startState:
                startState();
                break;
            case State::homeState:
                homeState();
                break;
            case State::stackState:
                // pickPlaceState(availableIngredients[0].position, currentState);
                break;
        }
        loop_rate.sleep();
    }

    // loop_rate.sleep();

    // feedback->status = "Halfway complete...";
    // goal_handle->publish_feedback(feedback);

    // loop_rate.sleep();
    // feedback->status = "Order complete!";
    // goal_handle->publish_feedback(feedback);

    // result->success = true;
    // goal_handle->succeed(result);
  }

  /********************STATE FUNCTIONS********************/
  /**
   * @brief home state only if ingredients are missing
   * @returns updates x, y position of camera (pos struct)
   */
  void homeState()
  {
    // TO DO
    // capture image from camera and process image to find available ingredients and their positions
    // 
  }
  /**
   * @brief Only runs at start, moves arm to home position and determines all available ingredients
   */
  void startState() {
    // move arm to home position
    send_arm_dest(HOME_POS);
  
    // get information from perception to determine all available ingredients and their positions
    // this->set_accept_cv(true);
    // //wait until perception is complete
    // RCLCPP_INFO(this->get_logger(), "Waiting for perception data...");
    // while (rclcpp::ok() && this->get_accept_cv()) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100)); // small sleep
    // }
    
    // for (const auto & ingredient : this->get_available_ingredients()) {
    //   RCLCPP_INFO(this->get_logger(), "Available ingredients: %s, X:, %f Y: %f , Z: %f", ingredient_to_string(ingredient.ingredient).c_str(), ingredient.position[0], ingredient.position[1], ingredient.position[2]);
    // }
    // this->currentState = State::homeState;
  }

  void pickPlaceState()
  {
    // TO DO:
    // pick ingredient at pos
    // close gripper
    // move to stack position
    // open gripper
  }

  /**
   * @brief Pick up ingredient and place it in the stack
   * @param pos x, y position of ingredient (pos struct)
   * @param currentState (enum class)
   * @returns updated position of ingredient (pos struct)
   */
  void stackState()
  {
    // TO DO:
    // get information from perception to determine all available ingredients and their positions
    // if not all ingredients are available, send feedback to user input node
  }

  /********************PERCEPTION CALLBACKS ********************************* */
  /**
   * @brief Callback function to process perceived ingredients
   * @param msg The message containing the list of perceived ingredients
   */
  void perception_callback(const custom_interfaces::msg::Ingredients::SharedPtr msg)
  {
    if (!this->get_accept_cv()) {
      RCLCPP_INFO(this->get_logger(), "Perception callback ignored because accept_cv is false");
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

    std::vector<Ingredients> missingIngredients = this->compareIngredients(order_ingredients, this->get_available_ingredients());
    if (missingIngredients.empty()) {
      this->set_accept_cv(false);
    } else {
      std::string missing_ingredients_str;
      for (const auto & ingredient : missingIngredients) {
        missing_ingredients_str += ingredient_to_string(ingredient) + " ";
      }
      this->input_feedback("Missing ingredients: " + missing_ingredients_str);
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

  /******************************* MoveIt Action Client Callbacks *************************** */
  
  /**
   * @brief response after requesting the goal position
   */
  void goal_response_callback(const GoalHandleMoveIt::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by movit_server");
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
    RCLCPP_INFO(this->get_logger(), feedback->status.c_str());
  }

  /**
   * @brief Callback function when result is received from action server
   * @param result The result from the action server
   */
  void result_callback(const rclcpp_action::ClientGoalHandle<custom_interfaces::action::Movement>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "TERMINATING...");
  }

  /**
   * @brief send the x, y, z, roll, pitch and yaw position to move the robot arm
   */
  void send_arm_dest(std::vector<double> pose) {
    if (!this->moveit_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = moveit_action_interface::Goal();
    goal_msg.command = "cartesian";
    goal_msg.constraints_identifier = "FULL";
    goal_msg.positions = pose;

    RCLCPP_INFO(this->get_logger(), "Sending order request to moveit...");

    this->moveit_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

   /************************HELPER FUNCTIONS********************/
  /**
   * @brief Compare available ingredients with order ingredients
   * @param orderIngredients (std::vector<Ingredients>)
   * @param availableIngredients (std::vector<itemPos>)
   * @returns list of missing ingredients
   */
  std::vector<Ingredients> compareIngredients(std::vector<Ingredients> orderIngredients, std::vector<IngredientPos> availableIngredients)
  {
    std::vector<Ingredients> missingIngredients;
    std::unordered_set<Ingredients> availableSet;
    for (const auto & a : availableIngredients) {
      availableSet.insert(a.ingredient);
    }

    // Check which order ingredients are missing
    for (const auto & ing : orderIngredients) {
      if (availableSet.count(ing) == 0) {
          missingIngredients.push_back(ing);
      }
    }

    return missingIngredients;
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
      RCLCPP_INFO(this->get_logger(), "Unknown ingredient string: %s", ingredient_str.c_str());
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
        return "topbun";
      case Ingredients::bottomBun:
        return "bottombun";
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
