# UGV Teleop Control System

A complete ROS2 Humble + micro-ROS system for controlling an Ackerman-steering UGV with ESP32.

## System Overview

### Hardware Components
- **ESP32 DOIT DevKit V1**: Low-level motor control and sensor reading
- **BTS7960 Motor Drivers (x2)**: One for steering, one for driving
- **Encoder Motor**: Steering motor with 600 PPM (50 pulses/degree)
- **Limit Switches (x2)**: Safety limits for steering rack (±10°)
- **Differential Drive**: Main driving system (no encoders)

### Software Architecture

```
┌─────────────────────────────────────────────┐
│           ROS2 Humble (Ubuntu 22.04)        │
│  ┌──────────────────┬────────────────────┐  │
│  │  Teleop Keyboard │  micro-ROS Agent   │  │
│  └────────┬─────────┴──────────┬─────────┘  │
│           │     /cmd_vel       │ Serial     │
│           └────────────────────┘            │
└─────────────────────────────────────────────┘
                      │ USB Serial
┌─────────────────────────────────────────────┐
│               ESP32 (micro-ROS)             │
│  ┌──────────────┬────────────┬────────────┐ │
│  │  Steering    │  Driving   │ Calibration│ │
│  │  Controller  │  Controller│  System    │ │
│  └──────┬───────┴─────┬──────┴─────┬──────┘ │
│         │  PID        │  Velocity  │  Auto  │
└─────────┼─────────────┼────────────┼────────┘
          │             │            │
    ┌─────┴─────┐  ┌────┴────┐  ┌───┴────┐
    │ Encoder   │  │ Driving │  │ Limit  │
    │ + Steering│  │  Motor  │  │Switches│
    │   Motor   │  │         │  │        │
    └───────────┘  └─────────┘  └────────┘
```

## Pin Configuration

| Component | Pin | GPIO |
|-----------|-----|------|
| Steering LPWM | GPIO 25 | PWM Channel 0 |
| Steering RPWM | GPIO 26 | PWM Channel 1 |
| Driving LPWM | GPIO 27 | PWM Channel 2 |
| Driving RPWM | GPIO 14 | PWM Channel 3 |
| Encoder A | GPIO 34 | Input only |
| Encoder B | GPIO 35 | Input only |
| Limit Left | GPIO 32 | Interrupt |
| Limit Right | GPIO 33 | Interrupt |

## Installation

### 1. ESP32 Firmware (PlatformIO)

```bash
# Install PlatformIO dependencies (will download micro-ROS)
cd /path/to/Ackerman_UGV
pio pkg install

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### 2. ROS2 Workspace

```bash
# Install micro-ROS agent
sudo apt update
sudo apt install ros-humble-micro-ros-agent

# Build ROS2 workspace
cd ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Quick Start

1. **Connect ESP32** to your laptop via USB

2. **Find serial port**:
   ```bash
   ls /dev/ttyUSB*  # or /dev/ttyACM*
   ```

3. **Update launch file** if needed (default is `/dev/ttyUSB0`):
   Edit `ros2_ws/src/ugv_teleop/launch/ugv_teleop.launch.py`

4. **Launch the system**:
   ```bash
   cd ros2_ws
   source install/setup.bash
   ros2 launch ugv_teleop ugv_teleop.launch.py
   ```

5. **Control the UGV** using keyboard:
   - `i` = Forward
   - `k` = Stop
   - `,` = Backward
   - `j` = Turn left
   - `l` = Turn right
   - `u/o/m/.` = Diagonal movements
   - `space` = Emergency stop
   - `q/z` = Increase/decrease max speeds

### Manual Launch (Individual Components)

**Terminal 1** - micro-ROS Agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Terminal 2** - Teleop Keyboard:
```bash
ros2 run ugv_teleop teleop_keyboard
```

## Features

### ESP32 Firmware

- **Automatic Calibration**: Runs on first boot, finds steering center using limit switches
- **Persistent Memory**: Saves calibration to EEPROM/Preferences
- **PID Control**: Smooth steering angle control with encoder feedback
- **Safety Limits**: Hard ±10° steering limits to protect rack
- **Emergency Stop**: Triggered by limit switches or angle violations
- **100Hz Control Loop**: Fast, responsive motor control
- **10Hz Status Publishing**: Real-time status feedback to ROS2

### ROS2 System

- **cmd_vel Interface**: Standard Twist messages for compatibility with NAV2
- **Ackerman Conversion**: Automatic conversion from Twist to steering angle
- **Keyboard Teleop**: Intuitive WASD-style controls
- **Status Monitoring**: Real-time feedback on `/ugv/status` topic

## Calibration Process

1. **Power on UGV**
2. ESP32 checks for saved calibration in memory
3. If not found:
   - Moves steering left until limit switch triggers
   - Records encoder count
   - Moves right until opposite limit switch triggers
   - Records encoder count
   - Calculates center as midpoint
   - Moves to center and resets encoder to 0
   - Saves calibration to memory
4. Future boots load calibration from memory (no movement)

## Monitoring & Debugging

### View ROS2 Topics

```bash
# List all topics
ros2 topic list

# Monitor cmd_vel commands
ros2 topic echo /cmd_vel

# Monitor UGV status
ros2 topic echo /ugv/status

# Test cmd_vel manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### ESP32 Serial Monitor

```bash
pio device monitor
```

Expected output:
```
UGV ESP32 Controller Starting...
Calibration loaded from memory.
Initializing micro-ROS...
micro-ROS initialized successfully!
System ready!
```

## Troubleshooting

### micro-ROS Agent Connection Failed

**Symptoms**: "Failed to initialize micro-ROS. Retrying..."

**Solutions**:
1. Check USB cable connection
2. Verify serial port: `ls /dev/ttyUSB*`
3. Ensure micro-ROS agent is running
4. Check baud rate (must be 115200)
5. Try resetting ESP32

### Limit Switch Triggered

**Symptoms**: "EMERGENCY STOP: Limit switch activated!"

**Solutions**:
1. Power off UGV
2. Manually center steering
3. Power on and let recalibrate
4. Check limit switch wiring if false triggers

### Steering Not Responding

**Possible causes**:
- Calibration not complete
- Emergency stop active
- Encoder wiring issue
- Motor driver connections

**Debug**:
```bash
# Check status topic
ros2 topic echo /ugv/status

# Look for:
# - E-Stop: 0 (should be 0)
# - Angle and Target values changing
```

## Future Extensions

### NAV2 Integration (Planned)

The system is designed for easy NAV2 integration:

1. **Add ZED Camera**
2. **Implement Odometry** (when wheel encoders added)
3. **Create robot_state_publisher** with URDF
4. **Configure NAV2** parameters
5. **Enable Autonomous Navigation**

### File Structure

```
Ackerman_UGV/
├── platformio.ini          # ESP32 build config
├── include/                # ESP32 header files
│   ├── pin_config.h       # GPIO pin assignments
│   ├── MotorDriver.h      # BTS7960 driver class
│   ├── SteeringController.h  # Steering control + PID
│   └── DrivingController.h   # Velocity control
├── src/
│   └── main.cpp           # ESP32 firmware (micro-ROS)
└── ros2_ws/
    └── src/
        └── ugv_teleop/    # ROS2 teleop package
            ├── ugv_teleop/
            │   └── teleop_keyboard.py
            └── launch/
                └── ugv_teleop.launch.py
```

## Safety Notes

⚠️ **CRITICAL**: The steering rack has a maximum rotation of ±10°. Exceeding this **WILL damage the hardware**.

The system has multiple safety layers:
1. Software limits in steering controller
2. Hardware limit switches
3. Emergency stop on switch activation
4. Angle validation in encoder ISR

**Always**:
- Test with vehicle elevated first
- Keep emergency stop accessible
- Monitor serial output during initial tests
- Verify limit switches before operation

## Parameters

### Firmware Constants (pin_config.h)

- `MAX_STEERING_ANGLE_DEG`: 10.0° (hardware limit)
- `ENCODER_PPR`: 600 pulses/revolution
- `PULSES_PER_DEGREE`: 50 pulses/degree
- `PWM_FREQUENCY`: 1000 Hz
- `CONTROL_PERIOD_MS`: 10 ms (100Hz)

### ROS2 Parameters

- `speed`: Max linear velocity (default: 0.5 m/s)
- `turn`: Max angular velocity (default: 1.0 rad/s)

## License

MIT License (or specify your license)

## Author

Thasindu Wickrama

## Version

1.0.0 - Initial Release
