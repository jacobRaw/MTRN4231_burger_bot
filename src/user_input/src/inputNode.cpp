/**
 * 
 * @brief Entry point to the program for a user. Users can send service commands to the /input_topic
 *        and this node will setup an action client to communicate with the brain node action server
 *        to process the order and receive feedback for the user.
 * 
 * @launch ros2 launch user_input input.launch.py
 * @usage ros2 service call /input_server custom_interfaces/srv/InputServer "{username: "jacob", take_orders: true, order: "cheeseburger"}"
 * 
 * @author Jacob Rawung
*/ 

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/action/order_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/srv/input_server.hpp"


using std::placeholders::_1;

enum class BurgerType {
  CheeseBurger,
  VeggieBurger,
  DoublePattyBurger,
  LoadedBurger,
  LettuceBurger,
  ClassicBurger,
  Unknown
};

class InputNode : public rclcpp::Node
{
public:
  using action_interface = custom_interfaces::action::OrderRequest;
  using GoalHandleOrderRequest = rclcpp_action::ClientGoalHandle<action_interface>;

  explicit InputNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("InputNode", options), take_orders_(true)
  {
    RCLCPP_INFO(this->get_logger(), "InputNode has started.");
    
    // Create a server to receive commands
    srv_ = create_service<custom_interfaces::srv::InputServer>("input_server",
                                                               std::bind(&InputNode::server_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Create action server to communicate with brain node
    this->action_client_ptr_ = rclcpp_action::create_client<action_interface>(
        this,
        "user_interface"
    );

    using namespace std::placeholders;
    // Setup the goal options for the action client
    send_goal_options = rclcpp_action::Client<action_interface>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&InputNode::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&InputNode::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&InputNode::result_callback, this, _1);
  }

  /**
   * @brief Check if the node is still taking orders
   * @returns true if the node is taking orders, false otherwise
   */
  bool is_taking_orders() const
  {
    return this->take_orders_;
  }


private:
  rclcpp::Service<custom_interfaces::srv::InputServer>::SharedPtr srv_;
  rclcpp_action::Client<action_interface>::SharedPtr action_client_ptr_;
  rclcpp_action::Client<action_interface>::SendGoalOptions send_goal_options;
  bool take_orders_;

  /**************************input_topic SUBSCRIPTION*********************/

  /**
   * @brief Callback function for input_server service
   * @param request The request from the user using ROS2 CLI commands
   * @param response The response to be sent back to the user
   */
  void server_callback(const std::shared_ptr<custom_interfaces::srv::InputServer::Request> request,
                       std::shared_ptr<custom_interfaces::srv::InputServer::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "User Name: '%s'", request->username.c_str());
    RCLCPP_INFO(this->get_logger(), "Take Orders: '%s'", request->take_orders ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Order: '%s'", request->order.c_str());
    // generate the ingredient list based on the order

    if (!request->take_orders) {
      this->take_orders_ = false;
      response->success = true;
      response->message = "No longer taking orders.";
      RCLCPP_INFO(this->get_logger(), "No longer taking orders, TERMINATING...");
      return;
    }

    std::string order = std::string(request->order);
    //convert to lowercase
    std::transform(order.begin(), order.end(), order.begin(), ::tolower);
    BurgerType burger_type = string_to_burger_type(order);
    std::vector<std::string> ingredients = create_ingredient_list(burger_type);
    if (ingredients.empty()) {
      response->success = false;
      response->message = "Unknown order: " + order + " Accepted orders are cheeseburger, veggieburger, doublepattyburger, loadedburger, lettuceburger.";
      return;
    }

    bool result = send_order_request(ingredients);
    if (!result) {  
      response->success = false;
      response->message = "Failed to send order request to brain node. Action server may be unavailable.";
      return;
    }

    response->success = true;
    response->message = "Order received successfully.";
  }

  /**************************ACTION SERVER HANDLERS*********************/

  /**
   * @brief Callback function when goal request is sent to action server
   * @param goal_handle The goal handle to determine if the goal was accepted or rejected
   */
  void goal_response_callback(const GoalHandleOrderRequest::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  /**
   * @brief Callback function for feedback from action server
   * @param goal_handle Not used
   * @param feedback The feedback from the action server
   */
  void feedback_callback(
    GoalHandleOrderRequest::SharedPtr,
    const std::shared_ptr<const custom_interfaces::action::OrderRequest::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), feedback->status.c_str());
  }

  /**
   * @brief Callback function when result is received from action server
   * @param result The result from the action server
   */
  void result_callback(const rclcpp_action::ClientGoalHandle<custom_interfaces::action::OrderRequest>::WrappedResult & result)
  {
    // this->take_orders_ = false;
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

    RCLCPP_INFO(this->get_logger(), "Waiting for next order...");
  }

  /********************HELPER FUNCTIONS ********************/
  /**
   * @brief Create ingredient list based on order
   * @param order The order string from the user
   * @returns vector of ingredient strings returns empty vector if order is unknown
   */
  std::vector<std::string> create_ingredient_list(BurgerType order)
  {
    std::vector<std::string> ingredients = std::vector<std::string>();
    switch (order) {
      case BurgerType::CheeseBurger:
        ingredients = {"bun_bottom", "patty", "cheese", "bun_top"};
        break;
      case BurgerType::ClassicBurger:
        ingredients = {"bun_bottom", "patty", "lettuce", "tomato", "cheese", "bun_top"};
        break;
      case BurgerType::VeggieBurger:
        ingredients = {"bun_bottom", "lettuce", "tomato", "pickles", "bun_top"};
        break;
      case BurgerType::DoublePattyBurger:
        ingredients = {"bun_bottom", "patty", "patty", "lettuce", "tomato","cheese", "bun_top"};
        break;
      case BurgerType::LoadedBurger:
        ingredients = {"bun_bottom", "patty", "patty", "lettuce", "lettuce", "tomato", "cheese", "pickles", "bun_top"};
        break;
      case BurgerType::LettuceBurger:
        ingredients = {"lettuce", "patty", "cheese", "lettuce"};
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown Burger Type");
        break;
    }
    return ingredients;
  }

  /**
   * @brief Send order request to brain node via action server
   * @param user_name The name of the user placing the order
   * @param order The order details
   */
  bool send_order_request(const std::vector<std::string> & ingredients)
  {
    if (!this->action_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->take_orders_ = false;
      return false;
    }

    auto goal_msg = action_interface::Goal();
    goal_msg.ingredients = ingredients;

    RCLCPP_INFO(this->get_logger(), "Sending order request to brain node...");

    this->action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  /**
   * @brief Convert string to BurgerType enum
   * @param burger_str The burger type as a string
   * @returns Corresponding BurgerType enum value
   */
  BurgerType string_to_burger_type(const std::string & burger_str)
  { 
    if (burger_str == "classicburger") {
      return BurgerType::ClassicBurger;
    } else if (burger_str == "cheeseburger") {
      return BurgerType::CheeseBurger;
    } else if (burger_str == "veggieburger") {
      return BurgerType::VeggieBurger;
    } else if (burger_str == "doublepattyburger") {
      return BurgerType::DoublePattyBurger;
    } else if (burger_str == "loadedburger") {
      return BurgerType::LoadedBurger;
    } else if (burger_str == "lettuceburger") {
      return BurgerType::LettuceBurger;
    } else {
      return BurgerType::Unknown;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto input_node = std::make_shared<InputNode>();
  // will continuously allow for back to back orders until is_goal_done is true
  while (input_node->is_taking_orders()) {
    rclcpp::spin_some(input_node);
  }

  rclcpp::shutdown();
  return 0;
}
