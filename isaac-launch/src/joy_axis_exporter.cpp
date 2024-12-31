#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>


namespace util
{
    template <typename T>
    struct identity
    {
        typedef T type;
    };

    template <typename T>
    void declare_param(rclcpp::Node * node, const std::string param_name, T & param,
                       const typename identity<T>::type & default_value)
    {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }
};

class ExporterNode : public rclcpp::Node
{
public:
    ExporterNode() : Node("joy_repub_node")
    {
        util::declare_param(this, "axis_num", this->param.axis_num, 0);
        util::declare_param(this, "value_scalar", this->param.scalar, 1.);

        this->val_pub = this->create_publisher<std_msgs::msg::Float64>(
            "joy_axis_value", 1);
        this->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SensorDataQoS{},
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& joy)
            {
                if(joy->axes.size() > static_cast<size_t>(this->param.axis_num))
                {
                    std_msgs::msg::Float64 val;
                    val.data = static_cast<double>(joy->axes[this->param.axis_num]) * this->param.scalar;
                    // RCLCPP_INFO(this->get_logger(), "DEBUG: %f", val.data);
                    this->val_pub->publish(val);
                }
            }
        );
    }
    ~ExporterNode() = default;

protected:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr val_pub;

    struct
    {
        int axis_num;
        double scalar;
    }
    param;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExporterNode>());
    rclcpp::shutdown();

    return 0;
}
