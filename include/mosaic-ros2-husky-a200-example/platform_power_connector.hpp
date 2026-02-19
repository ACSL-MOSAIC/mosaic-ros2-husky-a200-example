//
// Created by yhkim on 2/20/26.
//

#ifndef MOSAIC_HUSKY_A200_PLATFORM_POWER_CONNECTOR_HPP
#define MOSAIC_HUSKY_A200_PLATFORM_POWER_CONNECTOR_HPP

#include "mosaic-ros2-base/configurer/ros2_a_dc_handler_configurer.hpp"
#include <mosaic/handlers/data_channel/data_channel_sendable.hpp>

#include "clearpath_platform_msgs/msg/power.hpp"

namespace mosaic::ros2::clearpath::connector {
    class PlatformPowerConnectorConfigurer : public ROS2ADCHandlerConfigurer {
    public:
        PlatformPowerConnectorConfigurer() = default;

        std::string GetConnectorType() const override {
            return "ros2-clearpath-sender-clearpath_platform-Power-A200";
        }

        void Configure() override;

        void Callback(clearpath_platform_msgs::msg::Power::SharedPtr msg);

    private:
        std::shared_ptr<rclcpp::Subscription<clearpath_platform_msgs::msg::Power> > subscription_;
    };

    class PlatformPowerDataChannel : public handlers::DataChannelSendable {
    public:
        explicit PlatformPowerDataChannel(const std::string &channel_name) : DataChannelSendable(channel_name) {}

        ~PlatformPowerDataChannel() override = default;

        void OnPowerReceived(const clearpath_platform_msgs::msg::Power::SharedPtr &power);

    };
}  // namespace mosaic::ros2::clearpath::connector

#endif //MOSAIC_HUSKY_A200_PLATFORM_POWER_CONNECTOR_HPP
