#include "nhk2026_canbridge.hpp"

#include "lifecycle_msgs/msg/transition.hpp"

#include <chrono>
#include <functional>
#include <utility>

using std::placeholders::_1;

CanBridgenhk2026::CanBridgenhk2026()
: rclcpp_lifecycle::LifecycleNode(std::string("nhk2026_canbridge"))
{
    std::vector<std::string> default_topic = {};
    this->declare_parameter("pub_float_bridge_topic", default_topic);
    this->declare_parameter("pub_int_bridge_topic", default_topic);
    this->declare_parameter("pub_bytes_bridge_topic", default_topic);
    this->declare_parameter("sub_float_bridge_topic", default_topic);
    this->declare_parameter("sub_int_bridge_topic", default_topic);
    this->declare_parameter("sub_bytes_bridge_topic", default_topic);

    std::vector<int> default_canid = {};
    this->declare_parameter("pub_float_bridge_canid", default_canid);
    this->declare_parameter("pub_int_bridge_canid", default_canid);
    this->declare_parameter("pub_bytes_bridge_canid", default_canid);
    this->declare_parameter("sub_float_bridge_canid", default_canid);
    this->declare_parameter("sub_int_bridge_canid", default_canid);
    this->declare_parameter("sub_bytes_bridge_canid", default_canid);
    
    this->declare_parameter("ifname", "can0");
    this->declare_parameter("add_cmd_vel", false);
    this->declare_parameter("add_cmd_vel_feedback", false);

    this->declare_parameter("cmd_vel_canid", 0x100);
    this->declare_parameter("cmd_vel_topic_name", "cmd_vel");
    this->declare_parameter("cmd_vel_feedback_canid", 0x101);
    this->declare_parameter("cmd_vel_feedback_topic_name", "cmd_vel_feedback");
}

CanBridgenhk2026::~CanBridgenhk2026()
{
    this->stop_bridge_();
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_configure(const rclcpp_lifecycle::State &state)
{
    this->pub_float_bridge_topic_list_ = this->get_parameter("pub_float_bridge_topic").as_string_array();
    this->pub_int_bridge_topic_list_ = this->get_parameter("pub_int_bridge_topic").as_string_array();
    this->pub_bytes_bridge_topic_list_ = this->get_parameter("pub_bytes_bridge_topic").as_string_array();
    this->sub_float_bridge_topic_list_ = this->get_parameter("sub_float_bridge_topic").as_string_array();
    this->sub_int_bridge_topic_list_ = this->get_parameter("sub_int_bridge_topic").as_string_array();
    this->sub_bytes_bridge_topic_list_ = this->get_parameter("sub_bytes_bridge_topic").as_string_array();

    const std::vector<int64_t> pub_float_canids = this->get_parameter("pub_float_bridge_canid").as_integer_array();
    const std::vector<int64_t> pub_int_canids = this->get_parameter("pub_int_bridge_canid").as_integer_array();
    const std::vector<int64_t> pub_bytes_canids = this->get_parameter("pub_bytes_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_float_canids = this->get_parameter("sub_float_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_int_canids = this->get_parameter("sub_int_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_bytes_canids = this->get_parameter("sub_bytes_bridge_canid").as_integer_array();

    this->pub_float_bridge_canid_list_.assign(pub_float_canids.begin(), pub_float_canids.end());
    this->pub_int_bridge_canid_list_.assign(pub_int_canids.begin(), pub_int_canids.end());
    this->pub_bytes_bridge_canid_list_.assign(pub_bytes_canids.begin(), pub_bytes_canids.end());
    this->sub_float_bridge_canid_list_.assign(sub_float_canids.begin(), sub_float_canids.end());
    this->sub_int_bridge_canid_list_.assign(sub_int_canids.begin(), sub_int_canids.end());
    this->sub_bytes_bridge_canid_list_.assign(sub_bytes_canids.begin(), sub_bytes_canids.end());

    const bool mismatch_pub_float = this->pub_float_bridge_topic_list_.size() != this->pub_float_bridge_canid_list_.size();
    const bool mismatch_pub_int = this->pub_int_bridge_topic_list_.size() != this->pub_int_bridge_canid_list_.size();
    const bool mismatch_pub_bytes = this->pub_bytes_bridge_topic_list_.size() != this->pub_bytes_bridge_canid_list_.size();
    const bool mismatch_sub_float = this->sub_float_bridge_topic_list_.size() != this->sub_float_bridge_canid_list_.size();
    const bool mismatch_sub_int = this->sub_int_bridge_topic_list_.size() != this->sub_int_bridge_canid_list_.size();
    const bool mismatch_sub_bytes = this->sub_bytes_bridge_topic_list_.size() != this->sub_bytes_bridge_canid_list_.size();

    if (mismatch_pub_float || mismatch_pub_int || mismatch_pub_bytes ||
        mismatch_sub_float || mismatch_sub_int || mismatch_sub_bytes)
    {
        if (mismatch_pub_float) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: pub_float_bridge_topic(size=%zu) != pub_float_bridge_canid(size=%zu)",
                this->pub_float_bridge_topic_list_.size(),
                this->pub_float_bridge_canid_list_.size());
        }
        if (mismatch_pub_int) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: pub_int_bridge_topic(size=%zu) != pub_int_bridge_canid(size=%zu)",
                this->pub_int_bridge_topic_list_.size(),
                this->pub_int_bridge_canid_list_.size());
        }
        if (mismatch_pub_bytes) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: pub_bytes_bridge_topic(size=%zu) != pub_bytes_bridge_canid(size=%zu)",
                this->pub_bytes_bridge_topic_list_.size(),
                this->pub_bytes_bridge_canid_list_.size());
        }
        if (mismatch_sub_float) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: sub_float_bridge_topic(size=%zu) != sub_float_bridge_canid(size=%zu)",
                this->sub_float_bridge_topic_list_.size(),
                this->sub_float_bridge_canid_list_.size());
        }
        if (mismatch_sub_int) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: sub_int_bridge_topic(size=%zu) != sub_int_bridge_canid(size=%zu)",
                this->sub_int_bridge_topic_list_.size(),
                this->sub_int_bridge_canid_list_.size());
        }
        if (mismatch_sub_bytes) {
            RCLCPP_ERROR(
                get_logger(),
                "Configuration mismatch: sub_bytes_bridge_topic(size=%zu) != sub_bytes_bridge_canid(size=%zu)",
                this->sub_bytes_bridge_topic_list_.size(),
                this->sub_bytes_bridge_canid_list_.size());
        }
        return CallbackReturn::FAILURE;
    }
    this->Ifname = this->get_parameter("ifname").as_string();
    this->add_cmd_vel = this->get_parameter("add_cmd_vel").as_bool();
    this->add_cmd_vel_feedback = this->get_parameter("add_cmd_vel_feedback").as_bool();

    this->cmd_vel_canid = this->get_parameter("cmd_vel_canid").as_int();
    this->cmd_vel_topic_name = this->get_parameter("cmd_vel_topic_name").as_string();
    this->cmd_vel_feedback_canid = this->get_parameter("cmd_vel_feedback_canid").as_int();
    this->cmd_vel_feedback_topic_name = this->get_parameter("cmd_vel_feedback_topic_name").as_string();    

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CanBridgenhk2026::parameters_callback, this, _1)
    );
    RCLCPP_INFO(
        get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_activate(const rclcpp_lifecycle::State &state)
{
    const bool mismatch_pub_float = this->pub_float_bridge_topic_list_.size() != this->pub_float_bridge_canid_list_.size();
    const bool mismatch_pub_int = this->pub_int_bridge_topic_list_.size() != this->pub_int_bridge_canid_list_.size();
    const bool mismatch_pub_bytes = this->pub_bytes_bridge_topic_list_.size() != this->pub_bytes_bridge_canid_list_.size();
    const bool mismatch_sub_float = this->sub_float_bridge_topic_list_.size() != this->sub_float_bridge_canid_list_.size();
    const bool mismatch_sub_int = this->sub_int_bridge_topic_list_.size() != this->sub_int_bridge_canid_list_.size();
    const bool mismatch_sub_bytes = this->sub_bytes_bridge_topic_list_.size() != this->sub_bytes_bridge_canid_list_.size();

    if (mismatch_pub_float || mismatch_pub_int || mismatch_pub_bytes ||
        mismatch_sub_float || mismatch_sub_int || mismatch_sub_bytes)
    {
        if (mismatch_pub_float)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between pub_float_bridge_topic_list_ (%zu) and "
                "pub_float_bridge_canid_list_ (%zu).",
                this->pub_float_bridge_topic_list_.size(),
                this->pub_float_bridge_canid_list_.size());
        }
        if (mismatch_pub_int)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between pub_int_bridge_topic_list_ (%zu) and "
                "pub_int_bridge_canid_list_ (%zu).",
                this->pub_int_bridge_topic_list_.size(),
                this->pub_int_bridge_canid_list_.size());
        }
        if (mismatch_pub_bytes)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between pub_bytes_bridge_topic_list_ (%zu) and "
                "pub_bytes_bridge_canid_list_ (%zu).",
                this->pub_bytes_bridge_topic_list_.size(),
                this->pub_bytes_bridge_canid_list_.size());
        }
        if (mismatch_sub_float)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between sub_float_bridge_topic_list_ (%zu) and "
                "sub_float_bridge_canid_list_ (%zu).",
                this->sub_float_bridge_topic_list_.size(),
                this->sub_float_bridge_canid_list_.size());
        }
        if (mismatch_sub_int)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between sub_int_bridge_topic_list_ (%zu) and "
                "sub_int_bridge_canid_list_ (%zu).",
                this->sub_int_bridge_topic_list_.size(),
                this->sub_int_bridge_canid_list_.size());
        }
        if (mismatch_sub_bytes)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Activation failed: size mismatch between sub_bytes_bridge_topic_list_ (%zu) and "
                "sub_bytes_bridge_canid_list_ (%zu).",
                this->sub_bytes_bridge_topic_list_.size(),
                this->sub_bytes_bridge_canid_list_.size());
        }
        return CallbackReturn::FAILURE;
    }
    try
    {
        this->can_bridge = std::make_unique<CanBridge>(this->Ifname);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return CallbackReturn::FAILURE;
    }

    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->float_publisher_.reserve(this->pub_float_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_float_bridge_canid_list_.size(); i++)
    {
        this->float_publisher_.push_back(
            this->create_publisher<std_msgs::msg::Float32MultiArray>(
                this->pub_float_bridge_topic_list_[i],
                device
            )
        );
    }
    this->float_subscribers_.reserve(this->sub_float_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_float_bridge_canid_list_.size(); i++)
    {
        this->float_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                this->sub_float_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::Float32MultiArray::ConstSharedPtr rxdata) {
                    this->float_sub_process(this->sub_float_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }

    this->int_publisher_.reserve(this->pub_int_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_int_bridge_canid_list_.size(); i++)
    {
        this->int_publisher_.push_back(
            this->create_publisher<std_msgs::msg::Int32MultiArray>(
                this->pub_int_bridge_topic_list_[i],
                device
            )
        );
    }
    this->int_subscribers_.reserve(this->sub_int_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_int_bridge_canid_list_.size(); i++)
    {
        this->int_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::Int32MultiArray>(
                this->sub_int_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::Int32MultiArray::ConstSharedPtr rxdata) {
                    this->int_sub_process(this->sub_int_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }

    this->bytes_publisher_.reserve(this->pub_bytes_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_bytes_bridge_canid_list_.size(); i++)
    {
        this->bytes_publisher_.push_back(
            this->create_publisher<std_msgs::msg::ByteMultiArray>(
                this->pub_bytes_bridge_topic_list_[i],
                device
            )
        );
    }
    this->bytes_subscribers_.reserve(this->sub_bytes_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_bytes_bridge_canid_list_.size(); i++)
    {
        this->bytes_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::ByteMultiArray>(
                this->sub_bytes_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::ByteMultiArray::ConstSharedPtr rxdata) {
                    this->bytes_sub_process(this->sub_bytes_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }

    if (this->add_cmd_vel)
    {
        this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            this->cmd_vel_topic_name,
            device,
            std::bind(&CanBridgenhk2026::cmd_vel_callback, this, _1)
        );
    }

    if (this->add_cmd_vel_feedback)
    {
        this->cmd_vel_feedback_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            this->cmd_vel_feedback_topic_name,
            device
        );
    }

    this->rx_error_.store(false);
    this->running_.store(true);
    this->rx_thread_ = std::thread([this] {this->rx_loop();});
    this->error_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CanBridgenhk2026::handle_rx_error, this)
    );

    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void CanBridgenhk2026::stop_bridge_() noexcept
{
    if (this->error_timer_)
    {
        this->error_timer_->cancel();
        this->error_timer_.reset();
    }
    this->rx_error_.store(false);
    this->float_subscribers_.clear();
    this->int_subscribers_.clear();
    this->bytes_subscribers_.clear();

    this->cmd_vel_subscriber.reset();

    this->running_.store(false);
    if (this->can_bridge)
    {
        try
        {
            this->can_bridge->shutdown();
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Exception during CAN bridge shutdown: %s",
                e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unknown exception during CAN bridge shutdown");
        }
    }
    if (this->rx_thread_.joinable()) this->rx_thread_.join();
    this->can_bridge.reset();

    this->float_publisher_.clear();
    this->int_publisher_.clear();
    this->bytes_publisher_.clear();

    this->cmd_vel_feedback_publisher.reset();
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->stop_bridge_();

    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->stop_bridge_();

    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_error(const rclcpp_lifecycle::State &state)
{
    this->stop_bridge_();

    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->stop_bridge_();

    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CanBridgenhk2026::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    const uint8_t st = this->get_current_state().id();
    if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        result.successful = false;
        result.reason = "can_bridge_node is active";
        return result;
    }

    result.successful = true;
    result.reason = "success";

    for (const rclcpp::Parameter &param : parameters)
    {
        const std::string &name = param.get_name();
        const rclcpp::ParameterType type = param.get_type();

        if (name == "pub_float_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_float_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_int_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_int_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_bytes_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_bytes_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_float_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_float_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_int_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_int_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_bytes_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_bytes_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }

        if (name == "pub_float_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_float_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_int_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_int_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_bytes_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_bytes_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_float_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_float_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_int_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_int_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_bytes_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_bytes_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }

        if (name == "ifname" && type == rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->Ifname = param.as_string();
            continue;
        }
        if (name == "add_cmd_vel" && type == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            this->add_cmd_vel = param.as_bool();
            continue;
        }
        if (name == "add_cmd_vel_feedback" && type == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            this->add_cmd_vel_feedback = param.as_bool();
            continue;
        }
        if (name == "cmd_vel_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->cmd_vel_canid = param.as_int();
            continue;
        }
        if (name == "cmd_vel_topic_name" && type == rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->cmd_vel_topic_name = param.as_string();
            continue;
        }
        if (name == "cmd_vel_feedback_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->cmd_vel_feedback_canid = param.as_int();
            continue;
        }
        if (name == "cmd_vel_feedback_topic_name" && type == rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->cmd_vel_feedback_topic_name = param.as_string();
            continue;
        }

    }

    return result;
}

void CanBridgenhk2026::rx_loop()
{
    while (rclcpp::ok() && this->running_.load() && this->can_bridge != nullptr)
    {
        CanBridge::RxData_struct rxdata;
        try
        {
            const bool got = this->can_bridge->receive_data(rxdata);
            if (!got)
            {
                continue;
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            this->rx_error_.store(true);
            this->running_.store(false);
            if (this->can_bridge) this->can_bridge->shutdown();
            break;
        }

        for (size_t i = 0; i < this->pub_float_bridge_canid_list_.size(); i++)
        {
            if (this->pub_float_bridge_canid_list_[i] == rxdata.canid)
            {
                std::vector<float> txdata_f = this->can_bridge->rxdata_to_float(rxdata);
                std_msgs::msg::Float32MultiArray txdata;
                txdata.data = txdata_f;
                this->float_publisher_[i]->publish(txdata);
                break;
            }
        }

        for (size_t i = 0; i < this->pub_int_bridge_canid_list_.size(); i++)
        {
            if (this->pub_int_bridge_canid_list_[i] == rxdata.canid)
            {
                std::vector<int> txdata_i = this->can_bridge->rxdata_to_int(rxdata);
                std_msgs::msg::Int32MultiArray txdata;
                txdata.data = txdata_i;
                this->int_publisher_[i]->publish(txdata);
                break;
            }
        }

        for (size_t i = 0; i < this->pub_bytes_bridge_canid_list_.size(); i++)
        {
            if (this->pub_bytes_bridge_canid_list_[i] == rxdata.canid)
            {
                std_msgs::msg::ByteMultiArray txdata;
                txdata.data = std::move(rxdata.data);
                this->bytes_publisher_[i]->publish(txdata);
                break;
            }
        }

        if (add_cmd_vel_feedback)
        {
            if (this->cmd_vel_feedback_canid == rxdata.canid)
            {
                std::vector<float> txdata_f = this->can_bridge->rxdata_to_float(rxdata);
                if (txdata_f.size() != 3)
                {
                    RCLCPP_WARN(this->get_logger(), "cmd_vel_feedback's payload is unexpected size");
                    continue;
                }
                geometry_msgs::msg::Twist txdata;
                txdata.linear.set__x(txdata_f[0]);
                txdata.linear.set__y(txdata_f[1]);
                txdata.angular.set__z(txdata_f[2]);
                if (this->cmd_vel_feedback_publisher != nullptr) this->cmd_vel_feedback_publisher->publish(txdata);
                continue;
            }
        }
    }
}

void CanBridgenhk2026::handle_rx_error()
{
    if (!this->rx_error_.load())
    {
        return;
    }
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        this->rx_error_.store(false);
        return;
    }

    RCLCPP_WARN(this->get_logger(), "rx_loop failed. deactivating lifecycle node.");
    this->rx_error_.store(false);
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void CanBridgenhk2026::float_sub_process(int canid, std_msgs::msg::Float32MultiArray::ConstSharedPtr rxdata)
{
    if (!can_bridge) return;
    try
    {
        this->can_bridge->send_float(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void CanBridgenhk2026::int_sub_process(int canid, std_msgs::msg::Int32MultiArray::ConstSharedPtr rxdata)
{
    if (!can_bridge) return;
    try
    {
        this->can_bridge->send_int(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void CanBridgenhk2026::bytes_sub_process(int canid, std_msgs::msg::ByteMultiArray::ConstSharedPtr rxdata)
{
    if (!can_bridge) return;
    try
    {
        this->can_bridge->send_bytes(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void CanBridgenhk2026::cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    std::vector<float> txdata_vector(3);
    txdata_vector[0] = rxdata->linear.x;
    txdata_vector[1] = rxdata->linear.y;
    txdata_vector[2] = rxdata->angular.z;

    try
    {
        this->can_bridge->send_float(this->cmd_vel_canid, txdata_vector);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<CanBridgenhk2026> node = std::make_shared<CanBridgenhk2026>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
