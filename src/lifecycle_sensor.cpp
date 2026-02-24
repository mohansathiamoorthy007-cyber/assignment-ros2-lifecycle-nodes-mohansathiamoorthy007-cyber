#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
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

    // 1. Configure: Setup the publisher
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_data", 10);
        RCLCPP_INFO(this->get_logger(), "Sensor configured");
        return CallbackReturn::SUCCESS;
    }

    // 2. Activate: Start timer and enable publisher
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        // Explicitly activate the publisher to allow data flow
        publisher_->on_activate();
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&LifecycleSensor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Sensor activated");
        return CallbackReturn::SUCCESS;
    }

    // 3. Deactivate: Stop the timer and disable publisher
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        timer_->cancel();
        publisher_->on_deactivate();

        RCLCPP_INFO(this->get_logger(), "Sensor deactivated");
        return CallbackReturn::SUCCESS;
    }

    // 4. Cleanup: Destroy resources
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        timer_.reset();
        publisher_.reset();

        RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // 5. Shutdown: Final cleanup
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(), "Sensor shutting down from state: %s", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
        publisher_->publish(msg);
    }

    // Use LifecyclePublisher instead of standard Publisher
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleSensor>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
