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
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_data", 10);
        RCLCPP_INFO(get_logger(), "Sensor configured");
        return CallbackReturn::SUCCESS;
    }

    // 2. Start timer and activate publisher
    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        // For LifecycleNodes, you MUST activate the publisher explicitly
        publisher_->on_activate();

        timer_ = this->create_wall_timer(
            500ms, std::bind(&LifecycleSensor::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Sensor activated");
        return CallbackReturn::SUCCESS;
    }

    // 3. Stop timer and deactivate publisher
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        publisher_->on_deactivate();

        RCLCPP_INFO(get_logger(), "Sensor deactivated");
        return CallbackReturn::SUCCESS;
    }

    // 4. Reset the publisher
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        publisher_.reset();
        RCLCPP_INFO(get_logger(), "Sensor cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // 5. Final shutdown log
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Sensor shutting down");
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        // Double check: don't publish if not in active state
        if (!publisher_->is_activated()) {
            return;
        }

        auto msg = std_msgs::msg::Float64();
        msg.data = dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
        publisher_->publish(msg);
    }

    // Use LifecyclePublisher for managed state behavior
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};
