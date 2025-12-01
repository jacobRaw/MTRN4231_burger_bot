#include <memory>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "custom_interfaces/msg/ingredients.hpp"
#include "custom_interfaces/msg/ingredient_pos.hpp"

using std::placeholders::_1;

class IngredientMarkerNode : public rclcpp::Node
{
public:
    IngredientMarkerNode() : Node("ingredient_marker_node")
    {
        
        ingredient_color_map_ = {
            {"bun_top",     {1.0f, 1.0f, 1.0f, 1.0f}}, // white
            {"bun_bottom",  {0.72f, 0.52f, 0.28f, 1.0f}}, // light brown
            {"lettuce",    {0.20f, 0.75f, 0.20f, 1.0f}}, //green
            {"tomato",     {0.90f, 0.15f, 0.20f, 1.0f}}, //red
            {"cheese",     {0.98f, 0.80f, 0.15f, 1.0f}}, //yellow
            {"patty",      {0.40f, 0.20f, 0.10f, 1.0f}}, //brown (meat colour)
            {"pickles",     {0.40f, 0.60f, 1.00f, 1.0f}} //blue
        };

        subscription_ = this->create_subscription<custom_interfaces::msg::Ingredients>(
            "/ingredients", 10,
            std::bind(&IngredientMarkerNode::ingredients_callback, this, _1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/ingredient_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Ingredient marker node started.");
    }

private:
    struct Color {
        float r, g, b, a;
    };

    std::unordered_map<std::string, Color> ingredient_color_map_;

    rclcpp::Subscription<custom_interfaces::msg::Ingredients>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void ingredients_callback(const custom_interfaces::msg::Ingredients::SharedPtr msg)
    {
        int id = 0;
        // RCLCPP_INFO(this->get_logger(), "Received %s, X: %f, Y: %f, Z: %f", msg->ingredients[0].ingredient.c_str(), msg->ingredients[0].pos[0], msg->ingredients[0].pos[1], msg->ingredients[0].pos[2]);
        for (const auto &ingredient : msg->ingredients)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "ingredient";
            marker.id = id++;
            if (ingredient.ingredient == "cheese") {
                marker.type = visualization_msgs::msg::Marker::CUBE;
            } else {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
            }
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position
            if (ingredient.pos.size() >= 3) {
                marker.pose.position.x = ingredient.pos[0];
                marker.pose.position.y = ingredient.pos[1];
                marker.pose.position.z = 0;
            }

            // Orientation (upright)
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Cylinder size
            marker.scale.x = 0.075;   // diameter
            marker.scale.y = 0.075;
            marker.scale.z = 0.02;   // height

            // Look up color
            Color c = {0.0f, 0.0f, 0.0f, 1.0f};  // default black
            auto it = ingredient_color_map_.find(ingredient.ingredient);
            if (it != ingredient_color_map_.end()) {
                c = it->second;
            }

            marker.color.r = c.r;
            marker.color.g = c.g;
            marker.color.b = c.b;
            marker.color.a = c.a;

            // Keep markers visible
            marker.lifetime = rclcpp::Duration::from_nanoseconds(2000000000); // 2 second

            marker_pub_->publish(marker);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IngredientMarkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
