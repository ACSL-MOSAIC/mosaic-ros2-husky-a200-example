//
// Created by yhkim on 2/20/26.
//

#include "mosaic-ros2-husky-a200-example/platform_power_connector.hpp"

using namespace mosaic::ros2::clearpath::connector;

void PlatformPowerConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 Clearpath clearpath_platform_msgs::Power Connector...");

    handler_ = std::make_shared<PlatformPowerDataChannel>(connector_config_->label);

    const auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    subscription_ = mosaic_node_->create_subscription<clearpath_platform_msgs::msg::Power>(
        connector_config_->params.at("topic_name"), qos,
        std::bind(&PlatformPowerConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);
    MOSAIC_LOG_INFO("Configuring ROS2 Clearpath clearpath_platform_msgs::Power Connector Done!");
}

void PlatformPowerConnectorConfigurer::Callback(clearpath_platform_msgs::msg::Power::SharedPtr msg) {
    MOSAIC_LOG_DEBUG("clearpath_platform_msgs::msg::Power received");

    if (handler_) {
        if (const auto power_handler = std::dynamic_pointer_cast<PlatformPowerDataChannel>(handler_)) {
            power_handler->OnPowerReceived(msg);
        }
    }
}

void PlatformPowerDataChannel::OnPowerReceived(const clearpath_platform_msgs::msg::Power::SharedPtr &power) {
    if (!Sendable()) {
        MOSAIC_LOG_DEBUG("PlatformPowerDataChannel Not Sendable!");
        return;
    }

    const auto now = std::chrono::high_resolution_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    Json::Value power_json;
    Json::Value voltages_json;
    voltages_json["battery_voltage"] = std::to_string(power->measured_voltages[0]) + "V";
    voltages_json["left_driver_voltage"] = std::to_string(power->measured_voltages[1]) + "V";
    voltages_json["right_driver_voltage"] = std::to_string(power->measured_voltages[2]) + "V";
    Json::Value currents_json;
    currents_json["mcu_and_user_port_current"] = std::to_string(power->measured_currents[0]) + "A";
    currents_json["left_driver_current"] = std::to_string(power->measured_currents[1]) + "A";
    currents_json["right_driver_current"] = std::to_string(power->measured_currents[2]) + "A";
    power_json["measured_voltages"] = voltages_json;
    power_json["measured_currents"] = currents_json;
    power_json["battery_connected"] = power->battery_connected;
    power_json["charger_connected"] = power->charger_connected;
    power_json["timestamp"] = timestamp;

    SendJson(power_json);
}

