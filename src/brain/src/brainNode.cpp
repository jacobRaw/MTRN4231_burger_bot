// UserInput node which reads in the userName and order from /input topic and appropriately 
// starts an action server with the brain to create the burger

#include <functional>
#include <memory>
#include <thread>

#include "custom_interfaces/action/order_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;

enum class State {
  homeState,
  verificationState,
  pickPlaceState,
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
  BrainNode()
  : Node("brainNode")
  {
    RCLCPP_INFO(this->get_logger(), "BrainNode has started.");
    // Initialize the state machine
    // State currentState = State::homeState;
    // create a server to receiver oders from the input node
  }

  void run()
    {
        
        
        // Main loop to handle state transitions
        // while (rclcpp::ok()) {
        //     switch (currentState) {
        //         case State::homeState:
        //             homeState();
        //             break;
        //         case State::verificationState:
        //             // verifyState(availableIngredients[0].position);
        //             break;
        //         case State::pickPlaceState:
        //             // pickPlaceState(availableIngredients[0].position, currentState);
        //             break;
        //     }
        //     rclcpp::spin_some(shared_from_this());
        // }
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
    void homeState()
    {
        // TO DO:
        // move arm to home position
        // capture image from camera and process image to find available ingredients and their positions
        // 

    }

    /**
     * @brief Get close to ingredient to verify type of ingredient
     * @param pos x, y position of ingredient (pos struct)
     * @returns x, y position of ingredient if ingredient correct and high quality
     * @returns 
     */
    void verifyState(Pos /*ingredientPos*/)
    {
        // TO DO:
    }

    /**
     * @brief Pick up ingredient and place it in the stack
     * @param pos x, y position of ingredient (pos struct)
     * @param currentState (enum class)
     * @returns updated position of ingredient (pos struct)
     */
    void pickPlaceState()
    {
        // TO DO:
    }

    /**
     * @brief Compare available ingredients with order ingredients
     * @param orderIngredients (std::vector<Ingredients>)
     * @param availableIngredients (std::vector<itemPos>)
     */
    void compareIngredients()
    {
        // TO DO:
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrainNode>());
  rclcpp::shutdown();
  return 0;
}
