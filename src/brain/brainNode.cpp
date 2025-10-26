// UserInput node which reads in the userName and order from /input topic and appropriately 
// starts an action server with the brain to create the burger

#include <chrono>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/command.hpp"

using std::placeholders::_1;

enum class State {
  homeState,
  verificationState,
  pickPlaceState,
};

enum class Ingredients {
  topBun,
  bottomBun
  lettuce,
  tomato,
  cheese,
  patty,
  pickles
};

struct pos {
    double x;
    double y;
}

struct itemPos {
    std::string name;
    pos position;
};


class BrainNode : public rclcpp::Node
{
public:
  InputNode()
  : Node("InputNode")
  {
    RCLCPP_INFO(this->get_logger(), "BrainNode has started.");
  }

  void run()
    {
        // Initialize the state machine
        currentState = State::homeState;
        
        // Main loop to handle state transitions
        while (rclcpp::ok()) {
            switch (currentState) {
                case State::homeState:
                    homeState();
                    break;
                case State::verificationState:
                    verifyState(availableIngredients[0].position);
                    break;
                case State::pickPlaceState:
                    pickPlaceState(availableIngredients[0].position, currentState);
                    break;
            }
            rclcpp::spin_some(shared_from_this());
        }
    }

private:
    //currentState (enum class)
    //arm pose read from the robot arm topic
    // order ingredients (std::vector<Ingredients)
    // available ingredients (std::vector<itemPos>)
    // bin position (itemPos)
    // stack position (itemPos)
    // pickupZ (double)
    // double dropHeight = 0.5;

    /**
     * @brief home state where entire workspace is visible by camera
     * @returns updates x, y position of camera (pos struct)
     */
    homeState()
    {
        // TO DO:
    }

    /**
     * @brief Get close to ingredient to verify type of ingredient
     * @param pos x, y position of ingredient (pos struct)
     * @returns x, y position of ingredient if ingredient correct and high quality
     * @returns 
     */
    verifyState(pos ingredientPos)
    {
        // TO DO:
    }

    /**
     * @brief Pick up ingredient and place it in the stack
     * @param pos x, y position of ingredient (pos struct)
     * @param currentState (enum class)
     * @returns updated position of ingredient (pos struct)
     */
    pickPlaceState()
    {
        // TO DO:
    }

    /**
     * @brief Compare available ingredients with order ingredients
     * @param orderIngredients (std::vector<Ingredients>)
     * @param availableIngredients (std::vector<itemPos>)
     */
    compareIngredients()
    {
        // TO DO:
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputNode>());
  rclcpp::shutdown();
  return 0;
}
