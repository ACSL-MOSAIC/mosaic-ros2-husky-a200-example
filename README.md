# mosaic-ros2-husky-a200-example

A MOSAIC ROS2 connector example package for the Clearpath Husky A200 platform.
This package demonstrates how to implement a custom connector that subscribes to `clearpath_platform_msgs::msg::Power` and transmits data over a WebRTC data channel.

## Dependencies

| Package | Description |
|---|---|
| `mosaic-ros2-base` | MOSAIC ROS2 base interface |
| `mosaic-ros2-geometry` | MOSAIC geometry connectors |
| `mosaic-ros2-sensor` | MOSAIC sensor connectors |
| `clearpath_platform_msgs` | Clearpath platform message definitions |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select mosaic-ros2-husky-a200-example
source install/setup.bash
```

## Run

### Direct execution

```bash
ros2 run mosaic-ros2-husky-a200-example mosaic-ros2-husky-a200-example \
  --ros-args \
  -p mosaic_config:=/path/to/mosaic_config.yaml \
  -p mosaic_log_level:=info \
  -p webrtc_log_level:=none
```

### Using the launch file

```bash
ros2 launch mosaic-ros2-husky-a200-example mosaic_bringup_launch.py \
  mosaic_config:=/path/to/mosaic_config.yaml
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `mosaic_config` | `./mosaic_config.yaml` | Path to the MOSAIC configuration file |
| `mosaic_log_level` | `info` | MOSAIC log level (`debug`, `info`, `warning`, `error`) |
| `webrtc_log_level` | `none` | WebRTC log level (`none`, `verbose`, `info`, `warning`, `error`) |

## Connectors

### PlatformPowerConnector (`ros2-clearpath-sender-clearpath_platform-Power-A200`)

Subscribes to a `clearpath_platform_msgs::msg::Power` topic and sends the data as JSON over a WebRTC data channel.

**Output JSON format:**

```json
{
  "measured_voltages": {
    "battery_voltage": "24.0V",
    "left_driver_voltage": "24.0V",
    "right_driver_voltage": "24.0V"
  },
  "measured_currents": {
    "mcu_and_user_port_current": "1.0A",
    "left_driver_current": "1.0A",
    "right_driver_current": "1.0A"
  },
  "battery_connected": true,
  "charger_connected": false,
  "timestamp": 1234567890000
}
```

Configure the connector in `mosaic_config.yaml`:

```yaml
connectors:
  - type: ros2-clearpath-sender-clearpath_platform-Power-A200
    label: platform_power
    params:
      topic_name: /platform/power_status
```

## License

Apache-2.0