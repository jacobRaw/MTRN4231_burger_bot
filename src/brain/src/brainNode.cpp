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

#include "custom_interfaces/action/order_request.hpp"
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
  pickles
};

struct Pos {
    double x;
    double y;
};

struct ItemPos {
    std::string name;
    Pos position;
};


class BrainNode : public rclcpp::Node
{
public:

  using action_interface = custom_interfaces::action::OrderRequest;
  using GoalHandleOrderRequest = rclcpp_action::ServerGoalHandle<action_interface>;

  explicit BrainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("brainNode", options)
  {
    RCLCPP_INFO(this->get_logger(), "BrainNode has started.");
    // Initialize the state machine
    // State currentState = State::homeState;
    // create a server to receiver oders from the input node

    using namespace std::placeholders;

    // Create action server to communicate with user input node
    this->action_server_ = rclcpp_action::create_server<action_interface>(
      this,
      "user_interface",
      std::bind(&BrainNode::handle_goal_request, this, _1, _2),
      std::bind(&BrainNode::handle_cancel, this, _1),
      std::bind(&BrainNode::handle_accepted, this, _1));

    // Initialize class attributes
    currentState = State::startState;
  }

private:
  rclcpp_action::Server<action_interface>::SharedPtr action_server_;
  State currentState;
  //arm pose read from the robot arm topic
  // order ingredients (std::vector<Ingredients)
  // available ingredients (std::vector<itemPos>)
  // bin position (itemPos)
  // stack position (itemPos)
  // pickupZ (double)
  // double dropHeight = 0.5;

  /********************MAIN STATE LOOP********************/
  /**
   * @brief Main loop to handle state transitions
   */
  void run(const std::shared_ptr<GoalHandleOrderRequest> goal_handle)
  {
        
    RCLCPP_INFO(this->get_logger(), "Processing order...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<action_interface::Feedback>();
    auto result = std::make_shared<action_interface::Result>();

    rclcpp::Rate loop_rate(5);
    feedback->status = "Starting order...";
    goal_handle->publish_feedback(feedback);
    // Main loop to handle state transitions
    while (rclcpp::ok()) {
        switch (currentState) {
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
        rclcpp::spin_some(shared_from_this());
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
   * @brief home state where entire workspace is visible by camera assumes arm is in home position
   * @returns updates x, y position of camera (pos struct)
   */
  void homeState()
  {
    // TO DO:
    // capture image from camera and process image to find available ingredients and their positions
    // 

  }
  /**
   * @brief Only runs at start, moves arm to home position and determines all available ingredients
   */
  void startState() {
    // TO DO:
    // move arm to home position
    // get information from perception to determine all available ingredients and their positions
    // if not all ingredients are available, send feedback to user input node
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

  /************************HELPER FUNCTIONS********************/
  /**
   * @brief Compare available ingredients with order ingredients
   * @param orderIngredients (std::vector<Ingredients>)
   * @param availableIngredients (std::vector<itemPos>)
   */
  void compareIngredients()
  {
    // TO DO:
  }

  /********************ACTION SERVER HANDLERS************************ */

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
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BrainNode::run, this, _1), goal_handle}.detach();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrainNode>());
  rclcpp::shutdown();
  return 0;
}
