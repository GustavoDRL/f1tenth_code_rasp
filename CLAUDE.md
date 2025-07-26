# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2-based autonomous racing system for F1Tenth vehicles, optimized for Raspberry Pi 4B. The system controls a 1/10 scale RC car with VESC motor controller, servo steering via GPIO, and YDLIDAR X4 sensor integration.

## Essential Commands

### Build and Run
```bash
# Setup environment (required for each new terminal)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build the project
cd ~/Documents/f1tenth_code_rasp
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Launch complete system
ros2 launch f1tenth_control f1tenth_complete_system.launch.py

# Launch with joystick control
ros2 launch joy_converter launch_joy_ackerman_fixed.launch.py
```

### Testing and Validation
```bash
# Run tests for specific package
colcon test --packages-select <package_name>
colcon test-result --verbose

# Check node status
ros2 node list
ros2 topic list
ros2 topic echo /drive
ros2 topic hz /drive

# Monitor system performance
ros2 topic echo /diagnostics
```

### Development Tools
```bash
# Python linting (automatically configured)
colcon test --packages-select <package> --pytest-args -k lint

# View logs
ros2 run rqt_console rqt_console
```

## Architecture Overview

### Core Packages
- **f1tenth_control**: Main control node handling servo (GPIO PWM) and VESC integration
  - Publishes odometry with TF transforms
  - Implements safety limits and emergency stop
  - 50Hz control loop target
  
- **vesc-humble**: VESC motor controller driver suite
  - vesc_driver: Hardware interface via USB serial
  - vesc_ackermann: Converts Ackermann to VESC commands
  - Critical for motor control and speed feedback

- **joy_converter**: Input handling for joystick/keyboard
  - Supports 8BitDo controller and keyboard fallback
  - Publishes to /drive topic with safety features

### Key Topics and Namespaces
- Namespace: `ego_racecar`
- Control: `/drive` (ackermann_msgs/AckermannDriveStamped)
- Sensors: `/scan` (sensor_msgs/LaserScan)
- Odometry: `/ego_racecar/odom` (nav_msgs/Odometry)
- Diagnostics: `/diagnostics` (diagnostic_msgs/DiagnosticArray)

### Configuration
All system parameters are centralized in `config/system_config.yaml`:
- VESC settings (port, speed limits, current limits)
- Servo calibration (PWM values for steering angles)
- Control frequencies and timeouts
- Safety limits and transformations

### Performance Requirements
- Control loop: 50Hz (20ms max latency)
- Emergency stop: <5ms response time
- CPU usage: <80% on Raspberry Pi 4B
- Memory: <1.5GB total system usage

## Development Guidelines

### Code Standards
- Follow ROS2 Python style guide
- Use type hints for all functions
- Implement proper error handling with try-except blocks
- Add diagnostic messages for debugging
- Document safety-critical sections

### Real-time Considerations
- Avoid blocking operations in control loops
- Use ROS2 timers instead of sleep()
- Pre-allocate memory where possible
- Minimize garbage collection in hot paths
- Profile performance on actual Raspberry Pi hardware

### Hardware-Specific Notes
- GPIO operations require sudo or gpio group membership
- VESC connects via USB serial (typically /dev/ttyACM0)
- Servo control uses pigpio library for hardware PWM
- System optimized for Raspberry Pi 4B with 4GB RAM

### Safety Features
- Emergency stop on connection loss
- Speed and steering angle limits
- Watchdog timers for all critical nodes
- Diagnostic monitoring for system health
- Graceful degradation on component failure

## Common Issues and Solutions

1. **VESC Connection**: Check USB port permissions and cable connection
2. **Servo Not Moving**: Verify pigpio daemon is running (`sudo pigpiod`)
3. **High Latency**: Check CPU governor settings and disable unnecessary services
4. **Build Errors**: Ensure all dependencies installed via setup script

## Testing Approach

When implementing new features:
1. Test individual nodes in isolation first
2. Verify topic communication with `ros2 topic echo`
3. Check performance with `ros2 topic hz`
4. Monitor system resources during operation
5. Test emergency stop functionality
6. Validate on actual hardware before committing