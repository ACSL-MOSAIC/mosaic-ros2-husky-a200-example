//
// Created by yhkim on 2/20/26.
//

#include "mosaic-ros2-husky-a200-example/register.hpp"

#include <mosaic/auto_configurer/connector/connector_resolver.hpp>

#include "mosaic-ros2-husky-a200-example/platform_power_connector.hpp"

namespace mosaic::ros2::clearpath::connector {
    void RegisterConnectors() {
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<
            PlatformPowerConnectorConfigurer>();
    }
} // namespace mosaic::ros2::clearpath::connector
