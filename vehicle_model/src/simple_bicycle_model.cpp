#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class BicycleModelNode : public rclcpp::Node {
public:
    BicycleModelNode() : Node("bicycle_model_node") {
        // Initialize publishers for orientation, x, and y positions
        orientation_pub_ = create_publisher<std_msgs::msg::Float64>("/sim_output/orientation", 10);
        velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/sim_output/velocity", 10);
        x_position_pub_ = create_publisher<std_msgs::msg::Float64>("/sim_output/x_position", 10);
        y_position_pub_ = create_publisher<std_msgs::msg::Float64>("/sim_output/y_position", 10);
        

        // Initialize subscribers for acceleration and steering rate
        acceleration_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/sim_input/acceleration", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                // Update acceleration value
                acceleration_ = msg->data;
            });

        steering_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/sim_input/steering", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                // Update steering rate value
                steering_ = msg->data;
            });

        // Timer for simulating the bicycle model at a fixed rate
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BicycleModelNode::simulateBicycleModel, this));
    }

private:
    // Bicycle model parameters
    double velocity_ = 0.0; // Initial velocity
    double orientation_ = 0.0; // Initial orientation
    double x_position_ = 0.0; // Initial x position
    double y_position_ = 0.0; // Initial y position
    
    
    // Input values
    double acceleration_ = 0.0;
    double steering_ = 0.0;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr orientation_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_position_pub_; 
    

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr acceleration_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;

    // Timer for simulation
    rclcpp::TimerBase::SharedPtr timer_;

    // Function to simulate the bicycle model
    void simulateBicycleModel() {
        // Simple bicycle model simulation
        double dt = 0.05; // Time step
        double length = 2.0; // Bicycle length

        // 
        velocity_     += acceleration_ * dt;
        orientation_  += (velocity_ / length) * tan(steering_) * dt;
        x_position_   += velocity_ * cos(orientation_) * dt;
        y_position_   += velocity_ * sin(orientation_) * dt;


        // Publish simulated values
        auto orientation_msg = std::make_unique<std_msgs::msg::Float64>();
        orientation_msg->data = orientation_;
        orientation_pub_->publish(std::move(orientation_msg));

        auto x_position_msg = std::make_unique<std_msgs::msg::Float64>();
        x_position_msg->data = x_position_;
        x_position_pub_->publish(std::move(x_position_msg));

        auto y_position_msg = std::make_unique<std_msgs::msg::Float64>();
        y_position_msg->data = y_position_;
        y_position_pub_->publish(std::move(y_position_msg));

        auto velocity_msg = std::make_unique<std_msgs::msg::Float64>();
        velocity_msg->data = velocity_;
        velocity_pub_->publish(std::move(velocity_msg));

        
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BicycleModelNode>());
    rclcpp::shutdown();
    return 0;
}
