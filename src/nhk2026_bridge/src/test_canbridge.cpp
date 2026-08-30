#include "test_canbridge.hpp"

TestCanBridge::TestCanBridge()
: rclcpp_lifecycle::LifecycleNode(std::string("test_canbridge")),
Ifname("can0")
{
    this->declare_parameter<int>("vel_canid", 0x200);

    this->vel_canid = this->get_parameter("vel_canid").as_int();
    
    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TestCanBridge::parameters_callback, this, _1)
    );
}

TestCanBridge::CallbackReturn TestCanBridge::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

TestCanBridge::CallbackReturn TestCanBridge::on_activate(const rclcpp_lifecycle::State &state)
{
	try
	{
		this->bridge = std::make_unique<CanBridge>(this->Ifname);
	}
	catch(const std::exception& e)
	{
		RCLCPP_INFO(this->get_logger(), "please check can0");
		return CallbackReturn::FAILURE;
	}

    this->vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&TestCanBridge::vel_callback, this, _1)
    );

    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

TestCanBridge::CallbackReturn TestCanBridge::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->vel_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

TestCanBridge::CallbackReturn TestCanBridge::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->vel_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

TestCanBridge::CallbackReturn TestCanBridge::on_error(const rclcpp_lifecycle::State &state)
{
    this->vel_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

TestCanBridge::CallbackReturn TestCanBridge::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->vel_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void TestCanBridge::vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    if (!this->bridge)
    {
        RCLCPP_WARN(this->get_logger(),
                    "CAN bridge not initialized, dropping cmd_vel");
        return;
    }
    std::vector<float> txdata(3);
    txdata[0] = rxdata->linear.x;
    txdata[1] = rxdata->linear.y;
    txdata[2] = rxdata->angular.z;
    try
    {
        this->bridge->send_float(this->vel_canid, txdata);
        RCLCPP_INFO(this->get_logger(), "send!");
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "fail to write!");
        this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }   
}

rcl_interfaces::msg::SetParametersResult TestCanBridge::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "vel_canid" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->vel_canid = param.as_int();
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TestCanBridge> node = std::make_shared<TestCanBridge>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}