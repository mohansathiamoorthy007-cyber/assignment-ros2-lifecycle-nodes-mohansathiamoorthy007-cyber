#include <chrono>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// Define the CallbackReturn type for readability
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleSensor()
        : LifecycleNode("lifecycle_sensor"),
          gen_(rd_()),
          dist_(0.0, 100.0)
    {
        RCLCPP_INFO(this->get_logger(), "Lifecycle sensor node created");
    }

    // 1. Initialize publisher
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        // Note: create_publisher for LifecycleNodes returns a LifecyclePublisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_data", 10);
        RCLCPP_INFO(get_logger(), "Sensor configured");
        return CallbackReturn::SUCCESS;
    }

    // 2. Start timer and activate publisher
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        // Lifecycle publishers MUST be activated to actually send data
        publisher_->on_activate();

        timer_ = this->create_wall_timer(
            500ms, std::bind(&LifecycleSensor::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Sensor activated");
        return CallbackReturn::SUCCESS;
    }

    // 3. Stop timer and deactivate publisher
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        timer_.reset();
        publisher_->on_deactivate();

        RCLCPP_INFO(get_logger(), "Sensor deactivated");
        return CallbackReturn::SUCCESS;
    }

    // 4. Reset the publisher
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        publisher_.reset();
        RCLCPP_INFO(get_logger(), "Sensor cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // 5. Final shutdown log
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Sensor shutting down");
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        // Always good practice to check activation state
        if (!publisher_ || !publisher_->is_activated()) {
            return;
        }

        auto msg = std_msgs::msg::Float64();
        msg.data = dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
        publisher_->publish(msg);
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    // Lifecycle nodes need to use the NodeBaseInterface to be spun
    auto node = std::make_shared<LifecycleSensor>();
    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();
    return 0;
}
