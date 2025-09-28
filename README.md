# Hexapod Robot Simulator

A comprehensive 3D simulation of a hexapod (6-legged) robot with advanced kinematics, PID servo control, real-time animation, and interactive controls. This simulator demonstrates inverse kinematics, realistic servo dynamics, gait patterns, body articulation, idle positioning, and movement constraints with fast, responsive PID controllers.

![Python](https://img.shields.io/badge/python-v3.8+-blue.svg)
![Matplotlib](https://img.shields.io/badge/matplotlib-latest-green.svg)
![NumPy](https://img.shields.io/badge/numpy-latest-orange.svg)

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Command Line Interface](#command-line-interface)
- [Controls](#controls)
- [Technical Overview](#technical-overview)
- [Robot Specifications](#robot-specifications)
- [Kinematics](#kinematics)
- [Movement System](#movement-system)
- [Body Pose Control](#body-pose-control)
- [Idle Position System](#idle-position-system)
- [PID Servo Control System](#pid-servo-control-system)
- [Safety Features](#safety-features)
- [Code Structure](#code-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Features

### Core Functionality
- **Real-time 3D Animation**: Smooth 60fps visualization with matplotlib
- **PID Servo Control**: 18 individual PID controllers (3 per leg) for realistic servo dynamics
- **6-DOF Inverse Kinematics**: Each leg has 3 joints (coxa, femur, tibia)
- **Fast Response System**: High-speed servo movement (8 rad/s) with aggressive PID tuning
- **Tripod Gait Pattern**: Biologically-inspired alternating leg movement with faster cycle times
- **Interactive Controls**: Real-time keyboard input for all movements
- **Visual Feedback**: Purple trajectory lines show planned foot paths

### Movement Capabilities
- **Additive Movement System**: Combine multiple movements simultaneously
- **Walking**: Forward/backward locomotion with realistic gait
- **Crab Walking**: Sideways movement while maintaining body orientation
- **Turning**: Rotation around vertical axis
- **Body Articulation**: Independent pitch and yaw control
- **Height Adjustment**: Variable body height with safety constraints
- **Idle Position**: Predefined neutral stance for stable resting position

### Advanced Features
- **PID Servo Dynamics**: Realistic servo behavior with proportional-integral-derivative control
- **Fast Response Tuning**: Aggressive PID gains for snappy, responsive movement
- **Speed Limitations**: Configurable maximum servo speeds (currently 8 rad/s)
- **Joint-Specific Tuning**: Different PID parameters for coxa, femur, and tibia servos
- **Servo Limit Visualization**: Optional dotted reference lines showing 0° and 180° servo positions
- **Command-Line Interface**: Professional argument parsing with help system
- **Dynamic Trajectory Calculation**: Foot paths calculated from current positions
- **Ground Contact Maintenance**: Prevents feet from losing contact during rotation
- **Height Safety Limits**: Automatic prevention of impossible leg configurations
- **Target Position System**: Maintains foot placement during body rotation
- **World Coordinate System**: Proper separation of body and world reference frames

## Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Required Dependencies
```bash
pip install numpy matplotlib
```

### Optional Dependencies (for development)
```bash
pip install scipy  # For advanced mathematical operations
```

### Installation Steps
1. Clone or download the repository
2. Navigate to the project directory
3. Install dependencies: `pip install numpy matplotlib`
4. Run the simulation: `python hexaPodSim.py`

## Quick Start

```bash
# Navigate to project directory
cd /path/to/hexapod-simulator

# Run the normal simulation
python hexaPodSim.py

# Run with servo limit visualization (dotted reference lines)
python hexaPodSim.py -limit

# Show help and usage information
python hexaPodSim.py --help

# The 3D visualization window will open
# Use keyboard controls to move the robot
```

## Command Line Interface

The simulator supports command-line arguments for different visualization modes:

### Usage Options
```bash
python hexaPodSim.py              # Normal simulation mode
python hexaPodSim.py -limit       # Show servo limit reference lines  
python hexaPodSim.py --show-limits # Alternative long form
python hexaPodSim.py --help       # Display help information
```

### Servo Limit Visualization
When using the `-limit` flag, the simulator displays additional visual references:
- **Blue dotted lines**: Show 0° servo positions for each leg
- **Red dotted lines**: Show 180° servo positions for each leg
- **Semi-transparent**: Lines use 50% opacity to avoid visual clutter
- **Educational tool**: Helps understand servo movement constraints and workspace limits

### Help System
```bash
$ python hexaPodSim.py --help
usage: hexaPodSim.py [-h] [-limit]

Hexapod Robot Simulator with PID Control

options:
  -h, --help            show this help message and exit
  -limit, --show-limits Show dotted reference lines for 0° and 180° servo positions
```

## Controls

### Movement Controls (Additive - Can Be Combined)

| Key | Action | Description |
|-----|--------|-------------|
| `W` | Walk Forward | Move robot forward with tripod gait |
| `S` | Walk Backward | Move robot backward with tripod gait |
| `A` | Crab Walk Left | Sideways movement to the left |
| `D` | Crab Walk Right | Sideways movement to the right |
| `Q` | Turn Left | Rotate robot counterclockwise |
| `E` | Turn Right | Rotate robot clockwise |

### Stance Controls

| Key | Action | Description |
|-----|--------|-------------|
| `Z` | Idle Position | Move legs to neutral/idle stance (maintains body height) |

### Body Pose Controls

| Key | Action | Description |
|-----|--------|-------------|
| `I` | Pitch Up | Tilt robot body forward (nose down) |
| `K` | Pitch Down | Tilt robot body backward (nose up) |
| `J` | Yaw Left | Rotate robot body left (look left) |
| `L` | Yaw Right | Rotate robot body right (look right) |
| `M` | Reset Pose | Return body to neutral orientation |

### Height Controls

| Key | Action | Description |
|-----|--------|-------------|
| `↑` | Raise Body | Increase robot height (with safety limit) |
| `↓` | Lower Body | Decrease robot height |

### Joint Sweep Controls (Development/Testing)

| Key | Action | Description |
|-----|--------|-------------|
| `C` | Sweep Coxa | Animate coxa joints for testing |
| `G` | Sweep Femur | Animate femur joints for testing |
| `T` | Sweep Tibia | Animate tibia joints for testing |

### Control Examples

**Diagonal Walking**: Press `W` + `A` simultaneously to walk forward-left

**Turn While Walking**: Press `W` + `Q` to walk forward while turning left

**Look Around While Moving**: Press `W` + `J` to walk forward while looking left

**Complex Movement**: Press `W` + `D` + `E` + `L` to walk forward-right while turning right and looking right

**Return to Rest**: Press `Z` to move all legs to a stable idle position while maintaining current body height and orientation

## Technical Overview

### Architecture
The simulator is built with a modular architecture:

- **HexapodRobot Class**: Defines robot geometry and kinematics
- **Animation System**: Real-time 3D rendering with matplotlib
- **Control Handler**: Keyboard input processing and movement logic
- **Kinematics Engine**: Inverse kinematics calculations
- **Safety System**: Movement constraint validation

### Coordinate Systems
- **World Coordinates**: Fixed reference frame for the environment
- **Body Coordinates**: Robot-relative coordinate system
- **Leg Coordinates**: Individual leg reference frames

### Real-time Performance
- **60 FPS Animation**: Smooth visual feedback
- **Efficient Calculations**: Optimized matrix operations with NumPy
- **Memory Management**: Proper cleanup of visualization elements

## Robot Specifications

### Physical Dimensions
```python
Body Length:     2.0 units
Body Width:      1.0 units  
Body Height:     0.3 units
Number of Legs:  6 (3 per side)
```

### Leg Specifications
```python
Coxa Length:     0.5 units  # Hip joint to knee
Femur Length:    0.7 units  # Knee to ankle  
Tibia Length:    1.0 units  # Ankle to foot
Total Reach:     2.2 units  # Maximum leg extension
```

### Leg Layout
```
    0 --- 2 --- 4    (Right side: legs 0, 2, 4)
    |     |     |
    |  BODY     |
    |     |     |
    1 --- 3 --- 5    (Left side: legs 1, 3, 5)
    
Front → → → → → Back
```

### Joint Limits
- **Coxa**: ±180° (full rotation around vertical axis)
- **Femur**: 0° to 180° (0° = straight down, 180° = straight up)
- **Tibia**: -180° to 0° (negative angles point downward from femur)

## Kinematics

### Inverse Kinematics Algorithm
The simulator uses analytical inverse kinematics to calculate joint angles from desired foot positions:

1. **Coxa Angle**: `atan2(y, x)` - Direction to target
2. **Distance Calculation**: `D = sqrt((x² + y² - coxa²) + z²)`
3. **Femur Angle**: Law of cosines + geometric constraints
4. **Tibia Angle**: Law of cosines for 3-joint chain

### Mathematical Foundations
```python
# Coxa rotation
coxa_angle = atan2(target_y, target_x)

# Horizontal distance after coxa rotation  
r = sqrt(target_x² + target_y²) - coxa_length

# 3D distance to target
D = sqrt(r² + target_z²)

# Femur angle using law of cosines
cos_femur = (D² + femur² - tibia²) / (2 * D * femur)
femur_angle = atan2(z, r) + acos(cos_femur)

# Tibia angle using law of cosines  
cos_tibia = (femur² + tibia² - D²) / (2 * femur * tibia)
tibia_angle = acos(cos_tibia) - π
```

### Forward Kinematics
Joint angles are converted back to 3D positions for visualization:

```python
# Coxa end position
coxa_end = base + coxa_length * [cos(coxa), sin(coxa), 0]

# Femur end position  
femur_end = coxa_end + femur_length * [cos(coxa)*cos(femur), 
                                       sin(coxa)*cos(femur),
                                       -sin(femur)]

# Tibia end (foot) position
foot_pos = femur_end + tibia_length * [cos(coxa)*cos(femur+tibia),
                                       sin(coxa)*cos(femur+tibia), 
                                       -sin(femur+tibia)]
```

## Movement System

### Tripod Gait Pattern
The robot uses a tripod gait where legs move in two alternating groups:
- **Group A**: Legs 0, 2, 4 (right front, right middle, right back)
- **Group B**: Legs 1, 3, 5 (left front, left middle, left back)

### Gait Cycle
1. **Step Phase** (3 frames): Lift leg, move to new position, place down
2. **Stance Phase** (4 frames): Keep leg planted, move body forward
3. **Total Cycle**: 7 frames per complete step

### Trajectory Calculation
Foot trajectories are calculated dynamically from current positions:

```python
# Calculate movement vector
movement_vector = [forward_back, left_right, rotation_component]

# Generate arc trajectory
trajectory_points = []
for t in range(step_frames):
    # Horizontal movement (linear interpolation)
    x = start_x + (target_x - start_x) * t / frames
    y = start_y + (target_y - start_y) * t / frames
    
    # Vertical arc (parabolic)
    z = start_z + 4 * step_height * t * (frames - t) / frames²
    
    trajectory_points.append([x, y, z])
```

### Additive Movement System
Multiple movements can be combined by summing their components:

```python
# Calculate total movement vector
total_forward = move_forward - move_backward
total_sideways = move_right - move_left  
total_rotation = turn_right - turn_left

# Apply all movements simultaneously
combined_movement = [total_forward, total_sideways, total_rotation]
```

## Body Pose Control

### Rotation Matrices
Body orientation is controlled using 3D rotation matrices:

```python
# Pitch rotation (around Y axis)
R_pitch = [[cos(θ), 0, sin(θ)],
           [0,      1, 0     ],
           [-sin(θ), 0, cos(θ)]]

# Yaw rotation (around Z axis)  
R_yaw = [[cos(φ), -sin(φ), 0],
         [sin(φ),  cos(φ), 0],
         [0,       0,      1]]

# Combined rotation
R_total = R_yaw * R_pitch
```

### Target Position System
To maintain foot contact during body rotation:

1. **Store Target Positions**: World coordinates of desired foot positions
2. **Apply Body Rotation**: Rotate body and leg bases
3. **Inverse Kinematics**: Calculate joint angles to reach target positions
4. **Maintain Contact**: Feet stay planted while body articulates

### Body Pose Limits
- **Pitch Range**: ±30° (prevents extreme orientations)
- **Yaw Range**: ±45° (reasonable head movement)
- **Roll**: Currently disabled (could be added via `R` key)

## Idle Position System

### Overview
The idle position functionality provides a predefined neutral stance that places all legs in a stable, comfortable configuration. This feature is useful for:
- **Initialization**: Setting a known starting position
- **Rest State**: Providing a stable stance between movements  
- **Calibration**: Establishing a reference configuration
- **Recovery**: Returning to a safe position after complex movements

### Implementation
```python
# Predefined idle angles for each leg (degrees)
idle_angles_deg = [75, 75, 90, 90, 105, 105]  # Leg 0-5

# Calculate idle position for each leg
neutral_radius = coxa_length + femur_length * 0.8
sign = -1 if i % 2 == 0 else 1  # Right legs negative, left positive
angle = sign * idle_angles_rad[i]

# Position in world coordinates
idle_x = leg_base[0] + neutral_radius * cos(angle)
idle_y = leg_base[1] + neutral_radius * sin(angle)
idle_z = 0  # Maintain ground contact
```

### Key Features
- **Height Preservation**: Body height remains unchanged during idle positioning
- **Body Orientation**: Current pitch/yaw angles are maintained
- **Symmetric Stance**: Creates a stable, symmetric leg configuration
- **Ground Contact**: All feet remain in contact with the ground plane
- **Smooth Transition**: Uses standard inverse kinematics for natural movement

### Idle Leg Configuration
```
    0(75°) ---- 2(90°) ---- 4(105°)    (Right side)
        \         |         /
         \        |        /
          \    BODY      /
         /        |        \
        /         |         \
    1(75°) ---- 3(90°) ---- 5(105°)    (Left side)
```

The idle angles create a stable tripod-like stance with:
- **Front legs (0,1)**: 75° - Slightly forward and outward
- **Middle legs (2,3)**: 90° - Directly outward for maximum stability  
- **Rear legs (4,5)**: 105° - Slightly rearward and outward

## PID Servo Control System

### Overview
The hexapod simulator uses realistic PID (Proportional-Integral-Derivative) controllers to simulate actual servo motor behavior. Instead of instant position changes, each of the 18 servos (3 per leg × 6 legs) gradually moves toward target positions with realistic dynamics.

### PID Controller Features
- **Individual Control**: Each servo has its own PID controller with independent tuning
- **Fast Response**: Aggressive tuning for snappy, responsive movement
- **Speed Limits**: Realistic maximum servo speeds (8 radians/second)
- **Anti-Windup**: Integral term limiting prevents controller instability
- **Smooth Motion**: Natural acceleration and deceleration curves

### Servo Configuration

| Joint Type | Proportional (kp) | Integral (ki) | Derivative (kd) | Max Speed | Characteristics |
|------------|-------------------|---------------|-----------------|-----------|-----------------|
| **Coxa** | 5.0 | 0.5 | 0.2 | 8 rad/s | Fast hip rotation |
| **Femur** | 4.0 | 0.4 | 0.15 | 8 rad/s | Responsive upper leg |
| **Tibia** | 4.5 | 0.45 | 0.18 | 8 rad/s | Precise foot positioning |

### PID Implementation
```python
class PIDController:
    def update(self, dt):
        error = target_position - current_position
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        derivative_term = self.kd * derivative
        
        # Calculate output with speed limiting
        output = proportional + integral_term + derivative_term
        max_change = max_speed * dt
        position_change = np.clip(output * dt, -max_change, max_change)
        
        self.current_position += position_change
        return self.current_position
```

### Performance Characteristics
- **Fast Gait Cycle**: 4 frames per step (reduced from 7) for quicker walking
- **Responsive Controls**: Minimal delay between input and movement
- **Realistic Dynamics**: Overshooting, settling, and natural servo behavior
- **Stable Operation**: Tuned for fast response without oscillation

### Benefits of PID Control
1. **Realistic Movement**: Servos behave like actual hardware
2. **Smooth Transitions**: No instant "teleportation" to positions
3. **Natural Dynamics**: Acceleration, deceleration, and settling behavior
4. **Expandable Framework**: Ready for load simulation, backlash modeling
5. **Hardware Similarity**: Easy transition to real robot control

### Tuning Guidelines
- **Increase kp**: Faster response, but may cause overshoot
- **Increase ki**: Better steady-state accuracy, but may cause oscillation
- **Increase kd**: Reduced overshoot, smoother approach to target
- **Increase max_speed**: Faster movement, but may reduce stability
- **Reduce step_period**: Faster gait cycle for quicker walking

### Servo Limit Visualization
When enabled with the `-limit` command-line flag, the simulator displays servo movement boundaries:

```python
# Command to enable limit visualization
python hexaPodSim.py -limit

# Calculates limit positions for each leg
def calculate_servo_limit_positions(robot, leg_bases):
    for coxa_angle in [0, np.pi]:  # 0° and 180°
        # Calculate joint positions using forward kinematics
        coxa_end = base + coxa_length * [cos(angle), sin(angle), 0]
        # Default positions: femur=45°, tibia=-90°
        # Shows theoretical servo movement range
```

**Visual Elements:**
- **Blue dotted lines (b:)**: 0° servo positions - minimum range
- **Red dotted lines (r:)**: 180° servo positions - maximum range  
- **Transparency**: 50% alpha (alpha=0.5) for subtle reference
- **Line width**: 1 pixel for minimal visual interference

**Use Cases:**
- **Educational**: Understanding servo constraints and workspace limits
- **Debugging**: Identifying when servos approach movement boundaries
- **Design Analysis**: Validating leg configurations against servo capabilities
- **Workspace Visualization**: Seeing theoretical vs. actual reachable positions

## Safety Features

### Height Limitations
```python
max_safe_height = 0.94  # Maximum safe body height

# Calculate based on leg geometry
max_reach = coxa_length + femur_length + tibia_length - safety_margin
ground_clearance = current_body_height
available_reach = max_reach - ground_clearance

if requested_height > max_safe_height:
    print("Height limit reached - feet would lose ground contact!")
    # Prevent height increase
```

### Kinematic Constraints
- **Joint Limits**: Prevent impossible joint configurations
- **Reach Limits**: Ensure targets are within leg workspace  
- **Collision Avoidance**: Prevent leg-body intersections
- **Stability**: Maintain static stability during pose changes

### Error Handling
```python
# Clamp values to valid ranges
cos_value = np.clip(cos_value, -1.0, 1.0)

# Check for unreachable positions
if distance > max_leg_reach:
    # Scale down to reachable position
    scale_factor = max_leg_reach / distance
    target_position *= scale_factor
```

## Code Structure

### Main Components

```
hexaPodSim.py
├── Command Line Interface
│   ├── parse_arguments()   # Argument parsing with argparse
│   └── SHOW_SERVO_LIMITS   # Global flag for limit visualization
│
├── PID Controller Class
│   ├── __init__()          # PID parameter setup
│   ├── update()            # Position control with speed limits
│   └── reset()             # Controller state reset
│
├── HexapodRobot Class
│   ├── __init__()          # Robot geometry and PID setup
│   ├── set_servo_targets() # Set PID target positions
│   ├── update_servos()     # Update all PID controllers
│   └── leg_inverse_kinematics()  # IK calculations
│
├── Servo Limit Calculation
│   └── calculate_servo_limit_positions()  # 0° and 180° references
│
├── Event Handlers
│   ├── on_key_press()      # Handle key press events
│   └── on_key_release()    # Handle key release events
│
├── Animation System  
│   ├── init()              # Initialize visualization
│   ├── animate()           # Main animation loop with PID updates
│   └── Visualization       # 3D plotting with optional limit lines
│
└── Control Variables
    ├── Movement flags      # Forward, back, left, right, turn
    ├── Body pose          # Pitch, yaw, roll
    ├── PID controllers     # 18 servo controllers (3 per leg)
    └── Safety limits      # Height constraints
```

### Key Functions

**`HexapodRobot.__init__()`**
- Defines robot geometry and dimensions
- Calculates leg base positions
- Initializes 18 PID controllers with aggressive tuning
- Sets up initial joint angles and servo positions

**`PIDController.update(dt)`**
- Updates individual servo position using PID control
- Applies speed limits and anti-windup protection
- Returns smooth, realistic servo movement

**`robot.set_servo_targets(leg_index, coxa_target, femur_target, tibia_target)`**
- Sets target positions for all servos in a leg
- PID controllers work toward these targets gradually

**`robot.update_servos()`**
- Updates all 18 PID controllers simultaneously
- Returns current servo positions for visualization
- Applies realistic servo dynamics and speed limits

**`leg_inverse_kinematics(target_pos, base_pos, coxa_length, femur_length, tibia_length)`**
- Converts 3D target position to joint angles
- Uses analytical solution with law of cosines
- Returns (coxa_angle, femur_angle, tibia_angle)

**`parse_arguments()`**
- Processes command-line arguments using argparse module
- Supports -limit flag for servo limit visualization
- Provides --help option with usage information
- Returns parsed arguments for global configuration

**`calculate_servo_limit_positions(robot, leg_bases)`**
- Calculates theoretical servo limit positions (0° and 180°)
- Uses forward kinematics with default joint angles
- Returns coordinate arrays for dotted reference line visualization
- Only called when SHOW_SERVO_LIMITS is enabled

**`animate(frame)`**
- Main animation loop called 60 times per second
- Calculates target positions using inverse kinematics
- Updates PID controllers with new targets
- Renders legs using actual servo positions (not targets)
- Optionally draws servo limit reference lines when enabled
- Updates 3D visualization with realistic servo dynamics

**`on_key_press(event)` / `on_key_release(event)`**
- Handle keyboard input for real-time control
- Set movement flags for additive control system
- Implement safety checks for height limits

### Global Variables

```python
# Command line interface
args = parse_arguments()            # Parsed command-line arguments
SHOW_SERVO_LIMITS = args.show_limits # Global flag for limit visualization

# Movement control (additive system)
move_forward = [False]
move_backward = [False] 
move_left = [False]
move_right = [False]
turn_left = [False]
turn_right = [False]
move_to_idle = [False]  # Idle position control

# Body pose control
body_pitch = [0.0]  # Forward/backward tilt
body_yaw = [0.0]    # Left/right rotation
body_roll = [0.0]   # Side-to-side tilt

# Robot state
body_z = [0.15]     # Current body height
current_foot_positions = [None] * 6
target_foot_positions = [None] * 6

# PID servo control (18 controllers total)
robot.servo_controllers = [
    [coxa_pid, femur_pid, tibia_pid]  # 3 controllers per leg
    for leg in range(6)  # 6 legs
]

# Visualization elements (conditionally created)
servo_limit_lines = []  # Dotted reference lines (when -limit flag used)
```

## Troubleshooting

### Common Issues

**Problem: Robot legs appear twisted or distorted**
```
Solution: Check inverse kinematics calculations
- Verify target positions are within leg reach
- Ensure joint angle limits are respected
- Check for NaN values in calculations
```

**Problem: Feet don't follow purple trajectory lines**
```
Solution: Verify coordinate system consistency
- Ensure world coordinates used before body rotation
- Check that trajectory calculation matches movement logic
- Verify target position system is working correctly
```

**Problem: Robot falls through ground or flies away**
```
Solution: Check height and safety constraints
- Verify max_safe_height calculation
- Ensure ground contact is maintained
- Check for proper Z-axis calculations
```

**Problem: Jerky or unstable movement**
```
Solution: Review animation timing
- Verify frame rate is consistent (60 FPS target)
- Check for efficient calculations in animate() loop
- Ensure smooth interpolation in trajectories
```

**Problem: Keys not responding**
```
Solution: Check event handler setup
- Ensure matplotlib figure has focus
- Verify keyboard event connections
- Check for conflicting key bindings
```

### Performance Optimization

**For Better Frame Rates:**
```python
# Reduce calculation complexity
# Use vectorized NumPy operations
# Minimize matplotlib artist creation/deletion
# Cache rotation matrices when possible
# Optimize PID update frequency if needed
```

**For Faster Response:**
```python
# Increase PID gains (kp, ki, kd) for snappier movement
# Increase max_speed in PID controllers (currently 8 rad/s)
# Decrease step_period for faster gait cycle (currently 4)
# Reduce integral windup limits for faster correction
```

**For Smoother Animation:**
```python
# Decrease PID gains for gentler movement
# Reduce max_speed for slower servo movement
# Increase step_period for slower, smoother gaits
# Add trajectory smoothing between waypoints
```

### Debugging Tools

**Print Debug Information:**
```python
# Add to animate() function
if frame % 30 == 0:  # Every 0.5 seconds
    print(f"Frame: {frame}")
    print(f"Body height: {body_z[0]:.3f}")
    print(f"Movement: F{move_forward[0]} B{move_backward[0]} L{move_left[0]} R{move_right[0]}")
```

**Visualize Joint Angles:**
```python
# Uncomment angle text display in animate()
ax.text(coxa_end[0], coxa_end[1], coxa_end[2], 
        f'C:{np.degrees(coxa_angle):.1f}°', fontsize=8)
```

## Contributing

### Development Setup
1. Fork the repository
2. Create a feature branch
3. Make changes with proper comments
4. Test thoroughly with different movement combinations
5. Submit a pull request

### Coding Standards
- Follow PEP 8 Python style guidelines
- Use descriptive variable names
- Comment complex mathematical operations
- Include docstrings for new functions
- Maintain backward compatibility

### Areas for Enhancement

**Potential Improvements:**
- Add roll control for complete 6-DOF body pose
- Implement dynamic gait patterns (trot, bound, gallop)
- Expand servo limit visualization (femur and tibia angle limits)
- Add interactive limit adjustment via command-line parameters
- Include workspace boundary visualization and collision detection
- Add adaptive PID tuning based on load conditions
- Include servo backlash and deadband modeling
- Add joint torque/force calculations with PID load compensation
- Implement servo failure simulation and recovery
- Add real-time PID parameter adjustment interface
- Include servo temperature and current monitoring simulation
- Add save/load functionality for PID tuning profiles and limit configurations
- Implement path planning with PID trajectory optimization and limit awareness
- Add physics simulation (gravity, inertia, friction) affecting PID performance
- Create hardware interface for real servo control with limit monitoring
- Add configuration file support for different robot geometries and limits

**Bug Reports:**
Please include:
- Python version and OS
- Complete error messages
- Steps to reproduce the issue
- Expected vs actual behavior

### Testing Checklist

Before submitting changes, verify:
- [ ] All movement controls work individually
- [ ] Combined movements work correctly
- [ ] Command-line arguments work correctly (-limit, --help)
- [ ] Servo limit visualization displays properly when enabled
- [ ] Normal mode runs without limit lines when flag not used
- [ ] PID controllers respond appropriately to commands
- [ ] Servo movement is smooth without oscillation
- [ ] Body pose controls don't break leg kinematics
- [ ] Height limits prevent impossible configurations
- [ ] Trajectory visualization matches actual servo movement
- [ ] Servo limit lines don't interfere with main simulation
- [ ] PID gains provide stable, responsive control
- [ ] No runtime errors or warnings in both modes
- [ ] Smooth 60 FPS animation performance with PID updates and optional limits

---

## License

This project is open source and available under the MIT License.

## Acknowledgments

- Inspired by biological hexapod locomotion research
- Uses matplotlib for real-time 3D visualization
- Implements classic robotics inverse kinematics algorithms
- Thanks to the Python scientific computing community

---

*For questions, issues, or contributions, please refer to the project repository or contact the maintainers.*