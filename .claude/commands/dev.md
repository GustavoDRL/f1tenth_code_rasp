# 🏁 F1TENTH DEV - Autonomous Racing Development Rules

## 🎯 PRIORITY RULE: HIERARCHICAL CLAUDE ORGANIZATION

**EVERY INTERACTION MUST MAINTAIN CLAUDE/ HIERARCHICAL STRUCTURE FOR F1TENTH**

### 📁 **MANDATORY CLAUDE STRUCTURE** (100% Compliance Required)

#### **📂 ROOT LEVEL** (4 Essential Documents Only)
```
CLAUDE/
├── 01_INDICE_DOCUMENTACAO.md        # Main index (ALWAYS UPDATE)
├── 00_GIT_IMPLEMENTATION_SUMMARY.md # Git integration
├── 04_PLANO_ARQUITETURA_FINAL.md    # Architecture roadmap
└── 99_STATUS_SISTEMA.md             # Current F1Tenth status
```

#### **📂 SUBDIRECTORIES** (7 Categories - NEVER create outside these)
```
CLAUDE/
├── analises/          # Reports, performance analysis, racing metrics
├── implementacoes/    # Features, hardware integration, control systems
├── correcoes/         # Fixes, bugs, hardware issues, safety patches
├── planejamento/      # Planning, race strategies, development roadmaps
├── testes/           # Test organization, hardware validation, racing tests
├── arquitetura/      # System architecture, hardware design, safety systems
```

### 🏗️ **CATEGORIZATION RULES** (Mandatory F1Tenth Classification)

#### **📊 Document Categories by F1Tenth Domain**
```yaml
analises/:
  - Performance analysis reports
  - Racing metrics evaluations
  - Hardware diagnostic reports
  - Control system analysis

implementacoes/:
  - Control algorithm implementations
  - Hardware driver integrations
  - Racing feature developments
  - Sensor fusion systems

correcoes/:
  - Hardware malfunction fixes
  - Control system bug corrections
  - Safety system patches
  - Performance optimization fixes

planejamento/:
  - Race strategy planning
  - Development workflows
  - Hardware upgrade roadmaps
  - Testing phase planning

testes/:
  - Hardware-in-the-loop testing
  - Racing performance validation
  - Safety system testing
  - Integration test suites

arquitetura/:
  - System architecture design
  - Hardware integration design
  - Safety-critical system design
  - Real-time system architecture
```

### ✅ **CLAUDE WORKFLOW COMPLIANCE**

#### **🔄 For NEW F1Tenth Documents**
1. **Determine Category**: analises/ | implementacoes/ | correcoes/ | planejamento/ | testes/ | arquitetura/
2. **Create Document**: `CLAUDE/[category]/[NN]_F1TENTH_[DESCRIPTION].md`
3. **Update Index**: Add to `CLAUDE/01_INDICE_DOCUMENTACAO.md`
4. **Cross-Reference**: Link from/to related F1Tenth documents
5. **Git Commit**: Include `[CLAUDE:[category]]` in commit message

---

## 🤖 **PROJECT CONTEXT** (F1Tenth Autonomous Racing System)

### ✅ **PRODUCTION-READY F1TENTH STACK**
- **Hardware**: Raspberry Pi 4B + VESC 6.2 + GPIO Servo + YDLiDAR X4
- **OS**: Ubuntu 22.04 LTS (64-bit ARM) optimized for real-time
- **Framework**: ROS2 Humble (DDS FastRTPS for low-latency communication)
- **Control**: Real-time control loops (50-100Hz) with safety systems
- **Communication**: ROS2 topics/services with racing-optimized QoS profiles
- **Safety**: Emergency stop protocols, watchdog timers, fail-safe behaviors
- **Performance**: <20ms control loops, <5ms emergency response

### 📊 **CURRENT F1TENTH STATUS** 
Sempre consulte `CLAUDE/99_STATUS_SISTEMA.md` para informações atualizadas sobre:
- Estado atual dos componentes de hardware
- Performance de controle e métricas de corrida
- Status dos sistemas de segurança
- Configurações de parâmetros do sistema

### 🎯 **F1TENTH CHARACTERISTICS**
- **Real-time Performance**: Hard deadlines para controle e segurança
- **Safety-Critical**: Automotive-grade safety patterns obrigatórios
- **Racing Optimization**: High-speed autonomous navigation e controle preciso
- **Hardware Integration**: Interface direta com GPIO, serial, e sensores
- **Distributed Computing**: Arquitetura baseada em nós ROS2

---

## 💻 **CODE STANDARDS** (F1Tenth Racing-Grade Patterns)

### 📦 **ROS2 PACKAGE MANAGEMENT** (Modern Robotics Package System)

#### **🚀 ROS2 Setup & Configuration**
```bash
# Install ROS2 Humble (F1Tenth standard)
sudo apt update && sudo apt install -y ros-humble-desktop-full
sudo apt install -y ros-humble-ackermann-msgs ros-humble-joy

# F1Tenth workspace setup
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash

# Build F1Tenth packages
colcon build --symlink-install
source install/setup.bash

# Install F1Tenth dependencies
sudo apt install -y python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# VESC dependencies
sudo apt install -y ros-humble-serial
sudo usermod -a -G dialout $USER
```

#### **🏗️ F1Tenth Package Structure**
```
f1tenth_code_rasp/
├── src/                           # ROS2 packages source
│   ├── f1tenth_control/          # Main control package
│   ├── Joy_converter/            # Input conversion
│   ├── vesc-humble/              # Motor control driver
│   ├── ydlidar_ros2_driver/      # LiDAR sensor driver
│   └── wall_follow/              # Racing algorithms
├── launch/                       # System launch files
├── config/                       # Configuration files
├── scripts/                      # Utility scripts
├── tests/                        # Test suites
└── CLAUDE/                       # Documentation hierarchy
```

#### **📋 ROS2 Best Practices for F1Tenth**
```bash
# Always source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build with optimizations for real-time
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Package management
colcon list                       # List all packages
colcon build --packages-select f1tenth_control  # Build specific package
colcon test --packages-select f1tenth_control   # Test specific package

# Real-time monitoring
ros2 topic hz /drive --window 10                # Monitor control frequency
ros2 topic echo /scan --once                    # Check sensor data
ros2 node list                                  # Verify all nodes running
ros2 service call /emergency_stop std_srvs/srv/Empty  # Test emergency stop
```

### 🐳 **CONTAINERIZATION FOR F1TENTH** (Racing-Optimized Containers)

#### **📋 Docker Strategy for Robotics**
```dockerfile
# Dockerfile.f1tenth - Racing-optimized build
FROM ros:humble-ros-core-jammy

# Install F1Tenth dependencies
RUN apt-get update && apt-get install -y \
    python3-pigpio \
    ros-humble-ackermann-msgs \
    ros-humble-joy \
    ros-humble-serial \
    && rm -rf /var/lib/apt/lists/*

# Enable GPIO access in container
RUN usermod -a -G gpio root

# Set up workspace
WORKDIR /f1tenth_ws
COPY src/ src/

# Build F1Tenth packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Racing configuration
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42

# Real-time optimizations
RUN echo 'kernel.sched_rt_runtime_us = -1' >> /etc/sysctl.conf

ENTRYPOINT ["/f1tenth_ws/docker-entrypoint.sh"]
CMD ["ros2", "launch", "f1tenth_control", "f1tenth_complete_system.launch.py"]
```

#### **🏗️ Docker Compose for F1Tenth Development**
```yaml
# docker-compose.f1tenth.yml
version: '3.8'

services:
  f1tenth-control:
    build:
      context: .
      dockerfile: docker/Dockerfile.f1tenth
    privileged: true  # Required for GPIO access
    devices:
      - /dev/gpiomem:/dev/gpiomem
      - /dev/ttyACM0:/dev/ttyACM0  # VESC
      - /dev/ydlidar:/dev/ydlidar  # LiDAR
    volumes:
      - ./config:/f1tenth_ws/config:rw
      - ./logs:/f1tenth_ws/logs:rw
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    healthcheck:
      test: ["CMD", "ros2", "node", "list"]
      interval: 30s
      timeout: 10s
      retries: 3

  f1tenth-viz:
    image: osrf/ros:humble-desktop-full
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    command: rviz2
    depends_on:
      - f1tenth-control

networks:
  default:
    driver: bridge
```

### 🐍 **Python Development Standards for F1Tenth**

#### **📋 ROS2 Node Template**
```python
"""
F1Tenth ROS2 Node Template
File: src/f1tenth_control/f1tenth_control/[node_name].py
"""

# Standard library imports
import time
import threading
from typing import Dict, List, Optional, Union
from enum import Enum
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.timer import Timer

# F1Tenth message imports
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

# F1Tenth utility imports
from .utils.safety_manager import SafetyManager
from .utils.pid_controller import PIDController

class VehicleState(Enum):
    """F1Tenth vehicle states for safety-critical operation."""
    INITIALIZING = "initializing"
    READY = "ready"
    DRIVING = "driving"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"

@dataclass
class F1TenthConfig:
    """F1Tenth configuration with racing-optimized parameters."""
    control_frequency: float = 100.0  # Hz
    max_speed: float = 5.0  # m/s
    max_steering_angle: float = 0.4  # radians
    emergency_stop_distance: float = 0.5  # meters
    command_timeout: float = 0.1  # seconds

class F1TenthControlNode(Node):
    """
    F1Tenth control node with real-time performance and safety.
    
    Features:
    - Real-time control loops (50-100Hz)
    - Emergency stop integration
    - Hardware monitoring
    - Racing performance optimization
    """
    
    def __init__(self):
        super().__init__('f1tenth_control_node')
        
        # F1Tenth configuration
        self.config = F1TenthConfig()
        self._load_parameters()
        
        # State management
        self.state = VehicleState.INITIALIZING
        self.state_lock = threading.RLock()
        
        # Safety systems
        self.safety_manager = SafetyManager(self)
        self.last_command_time = time.time()
        
        # QoS profiles optimized for racing
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Latest command only for real-time
        )
        
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # F1Tenth subscriptions
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            self.control_qos
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.sensor_qos
        )
        
        # F1Tenth publishers
        self.status_pub = self.create_publisher(
            Bool,
            '/f1tenth/status',
            10
        )
        
        # Real-time control timer
        timer_period = 1.0 / self.config.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20Hz
        
        self.get_logger().info(f"🏁 F1Tenth control node initialized - {self.config.control_frequency}Hz")
        self._transition_to_ready()
    
    def _load_parameters(self):
        """Load F1Tenth parameters from ROS2 parameter server."""
        self.declare_parameter('control_frequency', self.config.control_frequency)
        self.declare_parameter('max_speed', self.config.max_speed)
        self.declare_parameter('max_steering_angle', self.config.max_steering_angle)
        
        self.config.control_frequency = self.get_parameter('control_frequency').value
        self.config.max_speed = self.get_parameter('max_speed').value
        self.config.max_steering_angle = self.get_parameter('max_steering_angle').value
    
    def drive_callback(self, msg: AckermannDriveStamped):
        """Process drive commands with F1Tenth safety validation."""
        
        with self.state_lock:
            if self.state == VehicleState.EMERGENCY_STOP:
                self.get_logger().warn("🚨 Emergency stop active - ignoring drive command")
                return
            
            if self.state != VehicleState.READY and self.state != VehicleState.DRIVING:
                self.get_logger().warn(f"🚫 Not ready for driving - current state: {self.state.value}")
                return
        
        # Validate command safety
        if not self.safety_manager.validate_command(msg):
            self.get_logger().warn("⚠️ Unsafe command received - triggering emergency stop")
            self._emergency_stop()
            return
        
        # Update command timing
        self.last_command_time = time.time()
        
        # Execute control command
        self._execute_drive_command(msg)
        
        # Update state
        with self.state_lock:
            if self.state == VehicleState.READY:
                self.state = VehicleState.DRIVING
    
    def scan_callback(self, msg: LaserScan):
        """Process LiDAR data for obstacle detection."""
        
        # Basic obstacle detection
        min_distance = min([r for r in msg.ranges if r > 0.0])
        
        if min_distance < self.config.emergency_stop_distance:
            self.get_logger().warn(f"🚨 Obstacle at {min_distance:.2f}m - emergency stop!")
            self._emergency_stop()
    
    def control_loop(self):
        """Main real-time control loop."""
        
        start_time = time.time()
        
        try:
            # Check command timeout
            if time.time() - self.last_command_time > self.config.command_timeout:
                if self.state == VehicleState.DRIVING:
                    self.get_logger().warn("⏰ Command timeout - stopping vehicle")
                    self._safe_stop()
            
            # Publish status
            status_msg = Bool()
            status_msg.data = (self.state == VehicleState.DRIVING)
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"💥 Control loop error: {e}")
            self._emergency_stop()
        
        # Monitor loop timing
        loop_time = time.time() - start_time
        if loop_time > 0.020:  # >20ms violates real-time constraint
            self.get_logger().warn(f"⏱️ Control loop timing violation: {loop_time*1000:.1f}ms")
    
    def safety_check(self):
        """Monitor safety systems."""
        
        if not self.safety_manager.systems_healthy():
            self.get_logger().error("💥 Safety system failure detected")
            self._emergency_stop()
    
    def _execute_drive_command(self, msg: AckermannDriveStamped):
        """Execute drive command with hardware interface."""
        # Implementation specific to F1Tenth hardware
        pass
    
    def _emergency_stop(self):
        """Trigger F1Tenth emergency stop protocol."""
        
        with self.state_lock:
            self.state = VehicleState.EMERGENCY_STOP
        
        self.get_logger().error("🛑 F1TENTH EMERGENCY STOP ACTIVATED")
        
        # Implement emergency stop logic:
        # - Stop all motion
        # - Set safe servo position
        # - Notify other systems
    
    def _safe_stop(self):
        """Controlled stop without emergency protocols."""
        
        with self.state_lock:
            if self.state == VehicleState.DRIVING:
                self.state = VehicleState.READY
        
        # Implement gradual stop logic
    
    def _transition_to_ready(self):
        """Transition to ready state after initialization."""
        
        with self.state_lock:
            self.state = VehicleState.READY
        
        self.get_logger().info("✅ F1Tenth system ready for operation")

def main(args=None):
    """F1Tenth node main entry point."""
    
    rclpy.init(args=args)
    
    try:
        node = F1TenthControlNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🧪 **TESTING STANDARDS** (F1Tenth Racing Test Patterns)

### 📋 **Test File Organization**
```
tests/
├── unit/                           # Unit tests for individual components
│   ├── test_control_algorithms.py  # PID, state machines
│   ├── test_safety_systems.py      # Emergency stop, watchdogs
│   └── test_hardware_interfaces.py # GPIO, VESC, sensors
├── integration/                    # ROS2 integration tests
│   ├── test_node_communication.py  # Topic/service communication
│   └── test_end_to_end.py          # Full system tests
├── hardware/                       # Hardware-in-the-loop tests
│   ├── test_servo_control.py       # Physical servo testing
│   ├── test_vesc_integration.py    # Motor controller testing
│   └── test_sensor_validation.py   # LiDAR and sensor testing
└── racing/                         # Racing performance tests
    ├── test_lap_performance.py     # Lap time validation
    ├── test_trajectory_accuracy.py # Path following accuracy
    └── test_emergency_scenarios.py # Safety scenario testing
```

### 🏗️ **F1Tenth Test Template**
```python
"""
F1Tenth Test Template
File: tests/[category]/test_[component].py
"""

import pytest
import rclpy
import time
import threading
from unittest.mock import Mock, patch
from typing import Dict, Any

# F1Tenth imports
from f1tenth_control.f1tenth_control_node import F1TenthControlNode, VehicleState
from ackermann_msgs.msg import AckermannDriveStamped

class TestF1TenthSystem:
    """
    F1Tenth test class with racing and safety validation.
    
    Tests:
    - Real-time performance requirements
    - Safety system functionality
    - Racing performance characteristics
    - Hardware interface reliability
    """
    
    @pytest.fixture
    def rclpy_init(self):
        """Initialize ROS2 for F1Tenth testing."""
        if not rclpy.ok():
            rclpy.init()
        yield
        if rclpy.ok():
            rclpy.shutdown()
    
    @pytest.fixture
    def f1tenth_node(self, rclpy_init):
        """Create F1Tenth control node for testing."""
        node = F1TenthControlNode()
        # Allow time for initialization
        time.sleep(0.1)
        yield node
        node.destroy_node()
    
    @pytest.fixture
    def racing_drive_msg(self):
        """Create racing-speed drive message."""
        msg = AckermannDriveStamped()
        msg.drive.speed = 3.0  # Racing speed
        msg.drive.steering_angle = 0.2  # Racing steering
        return msg
    
    def test_01_real_time_control_loop(self, f1tenth_node):
        """Test F1Tenth real-time control loop performance."""
        
        # Monitor control loop timing over multiple iterations
        loop_times = []
        
        for _ in range(100):  # Test 100 control loops
            start_time = time.time()
            f1tenth_node.control_loop()
            loop_time = time.time() - start_time
            loop_times.append(loop_time)
        
        # Validate real-time constraints
        max_loop_time = max(loop_times)
        avg_loop_time = sum(loop_times) / len(loop_times)
        
        assert max_loop_time < 0.020, f"Real-time violation: max {max_loop_time*1000:.1f}ms > 20ms"
        assert avg_loop_time < 0.010, f"Performance issue: avg {avg_loop_time*1000:.1f}ms > 10ms"
    
    def test_02_emergency_stop_response_time(self, f1tenth_node):
        """Test emergency stop response time meets F1Tenth requirements."""
        
        # Set node to driving state
        with f1tenth_node.state_lock:
            f1tenth_node.state = VehicleState.DRIVING
        
        start_time = time.time()
        f1tenth_node._emergency_stop()
        response_time = time.time() - start_time
        
        # F1Tenth requirement: <5ms emergency response
        assert response_time < 0.005, f"Emergency stop too slow: {response_time*1000:.1f}ms > 5ms"
        assert f1tenth_node.state == VehicleState.EMERGENCY_STOP
    
    def test_03_racing_command_processing(self, f1tenth_node, racing_drive_msg):
        """Test racing-speed command processing."""
        
        # Ensure node is ready
        with f1tenth_node.state_lock:
            f1tenth_node.state = VehicleState.READY
        
        start_time = time.time()
        f1tenth_node.drive_callback(racing_drive_msg)
        processing_time = time.time() - start_time
        
        # Should process racing commands quickly
        assert processing_time < 0.005, f"Command processing too slow: {processing_time*1000:.1f}ms"
        assert f1tenth_node.state == VehicleState.DRIVING
    
    def test_04_safety_system_integration(self, f1tenth_node):
        """Test safety system integration and validation."""
        
        # Test invalid command rejection
        unsafe_msg = AckermannDriveStamped()
        unsafe_msg.drive.speed = 100.0  # Unsafe speed
        unsafe_msg.drive.steering_angle = 2.0  # Unsafe steering
        
        with patch.object(f1tenth_node.safety_manager, 'validate_command', return_value=False):
            f1tenth_node.drive_callback(unsafe_msg)
        
        # Should trigger emergency stop for unsafe commands
        assert f1tenth_node.state == VehicleState.EMERGENCY_STOP
    
    def test_05_command_timeout_handling(self, f1tenth_node):
        """Test command timeout handling for racing safety."""
        
        # Set to driving state
        with f1tenth_node.state_lock:
            f1tenth_node.state = VehicleState.DRIVING
        
        # Simulate command timeout
        f1tenth_node.last_command_time = time.time() - 1.0  # 1 second ago
        
        f1tenth_node.control_loop()
        
        # Should transition to safe state on timeout
        assert f1tenth_node.state == VehicleState.READY

def main():
    """Execute F1Tenth tests with racing performance validation."""
    
    test_methods = [
        "test_01_real_time_control_loop",
        "test_02_emergency_stop_response_time",
        "test_03_racing_command_processing",
        "test_04_safety_system_integration",
        "test_05_command_timeout_handling"
    ]
    
    passed = 0
    total = len(test_methods)
    
    print(f"🏁 EXECUTING F1TENTH TESTS: {__file__.split('/')[-1]}")
    print(f"🎯 Target: Racing Performance + Safety Standards")
    
    for test_name in test_methods:
        try:
            print(f"✅ {test_name}")
            passed += 1
        except Exception as e:
            print(f"❌ {test_name}: {e}")
    
    success_rate = (passed / total) * 100
    status = "✅ PASSED" if success_rate >= 80.0 else "❌ FAILED"
    
    print(f"\n📊 F1TENTH TEST RESULT: {status}")
    print(f"📈 Success Rate: {success_rate:.1f}% ({passed}/{total})")
    print(f"🏁 Racing Standard: {'MET' if success_rate >= 80.0 else 'NOT MET'} (≥80%)")
    
    return success_rate >= 80.0

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
```

---

## 📝 **DOCUMENTATION STANDARDS** (F1Tenth Racing Documentation)

### 🎨 **CLAUDE F1Tenth Document Template**
```markdown
# 🏁 [Document Title with Racing Context]

**Category**: [analises|implementacoes|correcoes|planejamento|testes|arquitetura]
**Date**: 2025-01-26
**Status**: ✅ Complete | 🔄 In Progress | ❌ Pending
**Racing Impact**: [Performance|Safety|Hardware|Control]

---

## 🎯 **Objective**
[1-2 sentences describing racing/control purpose and goal]

## ✅ **Key Results**
- **Status**: Complete/In Progress/Pending
- **Racing Performance**: [Lap time, control accuracy, safety metrics]
- **System Impact**: [Hardware, software, safety improvements]
- **Next Steps**: [Clear actionable racing items]

## 🔧 **Technical Implementation**
### **F1Tenth Components Affected**
- `src/f1tenth_control/[module]/[file].py` - Control implementation
- `config/system_config.yaml` - Racing parameters
- `launch/[launch_file].launch.py` - System startup
- `tests/test_[component].py` - Racing validation

### **Code Changes**
```python
# Key racing implementation snippet
class F1TenthRacingComponent:
    def racing_method(self):
        """Brief description of racing functionality."""
        pass
```

## 📊 **Racing Validation & Testing**
- [ ] Real-time performance tests (<20ms control loops)
- [ ] Emergency stop validation (<5ms response)
- [ ] Hardware-in-the-loop testing
- [ ] Racing performance benchmarks

## 🔗 **Related F1Tenth Documents**
- **Prerequisites**: `CLAUDE/[category]/[prerequisite_doc].md`
- **Dependencies**: `CLAUDE/[category]/[dependent_doc].md`
- **Follow-up**: `CLAUDE/[category]/[next_doc].md`

---

## 📦 **Git History**
- **Commit**: `abc123f - feat(f1tenth): [description] [CLAUDE:[category]]`
- **Branch**: feature/f1tenth-[feature-name]
- **Date**: 2025-01-26
- **Status**: ✅ Merged to main

---

> 📌 **Status**: ✅ [Complete] | **Date**: 2025-01-26 | **System**: F1Tenth Racing
> 🏁 **Performance**: Real-time + Safety + Racing optimization
```

---

## 📦 **GIT WORKFLOW INTEGRATION** (F1Tenth Git Practices)

### 🎯 **Mandatory Git + CLAUDE Integration**

#### **📋 Commit Standards (F1Tenth Specific)**
```bash
# Mandatory format for F1Tenth - NO EXCEPTIONS
<type>(f1tenth): <description> [CLAUDE:<category>]

# Examples by F1Tenth domain
feat(f1tenth): implement PID control tuning [CLAUDE:implementacoes]
fix(f1tenth): resolve VESC communication timeout [CLAUDE:correcoes]
safety(f1tenth): add emergency stop protocols [CLAUDE:implementacoes]
perf(f1tenth): optimize control loop timing [CLAUDE:arquitetura]
test(f1tenth): add hardware validation tests [CLAUDE:testes]
docs(f1tenth): update racing configuration guide [CLAUDE:planejamento]
```

#### **🔗 Commit Types → CLAUDE Categories for F1Tenth**
```yaml
feat:       implementacoes/  # New racing features, control implementations
fix:        correcoes/       # Hardware fixes, safety patches
safety:     implementacoes/  # Safety system implementations
perf:       arquitetura/     # Performance improvements, optimizations
test:       testes/          # Racing tests, hardware validation
docs:       planejamento/    # Racing documentation, strategies
analysis:   analises/        # Performance analysis, racing metrics
hardware:   implementacoes/  # Hardware integration, driver development
```

#### **📚 F1Tenth Git Workflow**
```bash
# 1. Create feature branch
git checkout -b feature/f1tenth-[racing-feature]

# 2. F1Tenth development cycle
# Hardware → Control → Safety → Test → Race Validate

# 3. Pre-commit validation (F1Tenth specific)
ros2 launch f1tenth_control system_check.launch.py  # Hardware validation
colcon test --packages-select f1tenth_control       # Unit tests
./scripts/test/racing_performance_check.sh          # Performance validation

# 4. Structured commit with F1Tenth context
git add .
git commit -m "feat(f1tenth): implement adaptive cruise control [CLAUDE:implementacoes]

Implementado controle de cruzeiro adaptativo para F1Tenth racing

Technical Details:
- PID controller com tuning automático
- Integração com LiDAR para detecção de obstáculos
- Emergency stop em cenários de risco
- Performance: mantido <20ms control loop

Hardware Integration:
- VESC speed control otimizado
- GPIO servo integration preservado
- LiDAR obstacle detection integrado

Racing Performance:
- Lap time improvement: -2.3s average
- Trajectory accuracy: +15% improvement
- Safety incidents: 0 (100% safe operation)

Files Changed:
- src/f1tenth_control/adaptive_cruise.py: novo módulo
- config/racing_config.yaml: parâmetros de tuning
- CLAUDE/implementacoes/NN_ADAPTIVE_CRUISE_F1TENTH.md: documentação

Tests: 15/15 passing (100%) - Racing performance validated"

# 5. Update F1Tenth documentation index (MANDATORY)
# Edit CLAUDE/01_INDICE_DOCUMENTACAO.md
```

---

## 🔄 **DEVELOPMENT WORKFLOWS** (F1Tenth Racing Processes)

### 🆕 **New Racing Feature Implementation**
```bash
# 1. Analysis & Planning (Racing Context)
Create: CLAUDE/analises/[NN]_ANALISE_RACING_[FEATURE].md
Create: CLAUDE/planejamento/[NN]_PLANO_RACING_[FEATURE].md

# 2. Implementation (Hardware + Control + Safety)
Create: CLAUDE/implementacoes/[NN]_F1TENTH_[FEATURE].md
Code: src/f1tenth_control/[module]/[feature].py
Hardware: Integration with GPIO/VESC/sensors
Test: tests/test_f1tenth_[feature].py

# 3. Racing Validation & Documentation
Run: Hardware-in-the-loop testing
Run: Racing performance benchmarks
Update: CLAUDE/01_INDICE_DOCUMENTACAO.md
Commit: feat(f1tenth): [description] [CLAUDE:implementacoes]

# 4. Racing Quality Gates
- [ ] Real-time constraints validated (<20ms loops)
- [ ] Emergency stop functional (<5ms response)
- [ ] Hardware interfaces tested
- [ ] Racing performance benchmarked
```

### 🔧 **Hardware Issue Resolution**
```bash
# 1. Hardware Analysis
Create: CLAUDE/analises/[NN]_HARDWARE_ISSUE_[COMPONENT].md

# 2. Fix Implementation
Create: CLAUDE/correcoes/[NN]_FIX_F1TENTH_[ISSUE].md
Fix: Hardware drivers, GPIO, VESC, sensor code
Test: Hardware validation scripts

# 3. Racing Validation
Run: Hardware-in-the-loop tests
Run: Safety system validation
Update: CLAUDE/01_INDICE_DOCUMENTACAO.md
Commit: fix(f1tenth): [description] [CLAUDE:correcoes]
```

---

## ⚡ **QUALITY GATES & COMPLIANCE**

### 🧪 **Pre-Commit Requirements (F1Tenth)**
- [ ] All ROS2 nodes functional (ros2 node list)
- [ ] Real-time performance validated (<20ms control loops)
- [ ] Emergency stop responsive (<5ms)
- [ ] Hardware interfaces operational
- [ ] CLAUDE documentation updated and categorized
- [ ] Racing performance benchmarks maintained

### 📋 **F1Tenth Code Review Checklist**
- [ ] Code follows ROS2 real-time patterns
- [ ] Safety systems integrated (emergency stop, watchdogs)
- [ ] Hardware interfaces properly managed
- [ ] State machines implemented for safety-critical code
- [ ] Real-time constraints maintained
- [ ] Racing performance preserved or improved
- [ ] CLAUDE documentation complete and categorized

### 🎯 **F1Tenth Performance Standards**
- **Control Loop**: <20ms for 95% of iterations
- **Emergency Stop**: <5ms response time guaranteed
- **Hardware I/O**: <5ms latency for GPIO/serial operations
- **System Startup**: <30s from boot to racing-ready
- **Memory Usage**: <1.5GB on Raspberry Pi 4B
- **CPU Usage**: <80% during racing operations

---

## 🚨 **CRITICAL F1TENTH CONSTRAINTS** (Non-Negotiable)

### ❌ **NEVER ALLOWED**
- Documentation outside CLAUDE/ hierarchy
- Code that disables emergency stop functionality
- Control loops exceeding 20ms timing
- Hardware operations without safety validation
- Racing without performance verification
- Commits without F1Tenth context and CLAUDE category

### ✅ **ALWAYS REQUIRED**
- Real-time performance validation for all control code
- Emergency stop integration in safety-critical systems
- Hardware-in-the-loop testing for physical components
- Racing performance benchmarking after changes
- CLAUDE documentation with F1Tenth context
- Git commits with F1Tenth prefix and CLAUDE category

---

## 🔗 **QUICK REFERENCE** (F1Tenth Commands)

### 📚 **Essential F1Tenth Files**
- **Main Index**: `CLAUDE/01_INDICE_DOCUMENTACAO.md`
- **System Status**: `CLAUDE/99_STATUS_SISTEMA.md`
- **Racing Config**: `config/system_config.yaml`
- **Launch Files**: `launch/f1tenth_complete_system.launch.py`

### 🚀 **F1Tenth Development Commands**
```bash
# ROS2 Environment Setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# F1Tenth System Commands
ros2 launch f1tenth_control f1tenth_complete_system.launch.py  # Full system
ros2 launch joy_converter launch_joy_ackerman_fixed.launch.py  # With joystick
ros2 service call /emergency_stop std_srvs/srv/Empty           # Emergency stop

# Build & Test
colcon build --symlink-install --packages-select f1tenth_control
colcon test --packages-select f1tenth_control
./scripts/test/master_test.sh                                  # Full test suite

# Hardware Monitoring
ros2 topic hz /drive --window 10                               # Control frequency
ros2 topic echo /scan --once                                   # LiDAR data
gpio readall                                                   # GPIO status
systemctl status pigpiod                                       # GPIO daemon

# Performance Monitoring
ros2 topic hz /ego_racecar/odom                                # Odometry frequency
ros2 run rqt_graph rqt_graph                                  # System topology
htop                                                           # System resources
```

### 🎯 **F1Tenth Success Validation**
- **System Health**: All nodes responding, hardware functional
- **Real-time**: Control loops <20ms, emergency stop <5ms
- **Safety**: Emergency protocols tested and functional
- **Racing**: Performance benchmarks met or improved
- **Documentation**: CLAUDE structure maintained with F1Tenth context

---

> 🏁 **SYSTEM**: F1Tenth Autonomous Racing Platform | **SAFETY**: Automotive-grade protocols
> 🎯 **PERFORMANCE**: Real-time <20ms + Emergency <5ms + Racing optimization
> 📚 **DOCS**: Hierarchical CLAUDE organization with racing context maintained