# Hexapod Robot Simulator

A comprehensive 3D simulation of a hexapod (6-legged) robot with advanced kinematics, real-time animation, and interactive controls. This simulator demonstrates inverse kinematics, gait patterns, body articulation, and realistic movement constraints.

![Python](https://img.shields.io/badge/python-v3.8+-blue.svg)
![Matplotlib](https://img.shields.io/badge/matplotlib-latest-green.svg)
![NumPy](https://img.shields.io/badge/numpy-latest-orange.svg)

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Controls](#controls)
- [Technical Overview](#technical-overview)
- [Robot Specifications](#robot-specifications)
- [Kinematics](#kinematics)
- [Movement System](#movement-system)
- [Body Pose Control](#body-pose-control)
- [Safety Features](#safety-features)
- [Code Structure](#code-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Features

### Core Functionality
- **Real-time 3D Animation**: Smooth 60fps visualization with matplotlib
- **6-DOF Inverse Kinematics**: Each leg has 3 joints (coxa, femur, tibia)
- **Tripod Gait Pattern**: Biologically-inspired alternating leg movement
- **Interactive Controls**: Real-time keyboard input for all movements
- **Visual Feedback**: Purple trajectory lines show planned foot paths

### Movement Capabilities
- **Additive Movement System**: Combine multiple movements simultaneously
- **Walking**: Forward/backward locomotion with realistic gait
- **Crab Walking**: Sideways movement while maintaining body orientation
- **Turning**: Rotation around vertical axis
- **Body Articulation**: Independent pitch and yaw control
- **Height Adjustment**: Variable body height with safety constraints

### Advanced Features
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

# Run the simulation
python hexaPodSim.py

# The 3D visualization window will open
# Use keyboard controls to move the robot
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
├── HexapodRobot Class
│   ├── __init__()          # Robot geometry setup
│   └── leg_inverse_kinematics()  # IK calculations
│
├── Event Handlers
│   ├── on_key_press()      # Handle key press events
│   └── on_key_release()    # Handle key release events
│
├── Animation System  
│   ├── init()              # Initialize visualization
│   ├── animate()           # Main animation loop
│   └── Visualization       # 3D plotting and updates
│
└── Control Variables
    ├── Movement flags      # Forward, back, left, right, turn
    ├── Body pose          # Pitch, yaw, roll
    └── Safety limits      # Height constraints
```

### Key Functions

**`HexapodRobot.__init__()`**
- Defines robot geometry and dimensions
- Calculates leg base positions
- Sets up initial joint angles

**`leg_inverse_kinematics(target_pos, base_pos, coxa_length, femur_length, tibia_length)`**
- Converts 3D target position to joint angles
- Uses analytical solution with law of cosines
- Returns (coxa_angle, femur_angle, tibia_angle)

**`animate(frame)`**
- Main animation loop called 60 times per second
- Processes movement inputs and updates robot state
- Calculates new foot positions and joint angles
- Updates 3D visualization

**`on_key_press(event)` / `on_key_release(event)`**
- Handle keyboard input for real-time control
- Set movement flags for additive control system
- Implement safety checks for height limits

### Global Variables

```python
# Movement control (additive system)
move_forward = [False]
move_backward = [False] 
move_left = [False]
move_right = [False]
turn_left = [False]
turn_right = [False]

# Body pose control
body_pitch = [0.0]  # Forward/backward tilt
body_yaw = [0.0]    # Left/right rotation
body_roll = [0.0]   # Side-to-side tilt

# Robot state
body_z = [0.15]     # Current body height
current_foot_positions = [None] * 6
target_foot_positions = [None] * 6
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
```

**For Smoother Animation:**
```python
# Increase step_period for slower, smoother movements
# Use higher-order interpolation for trajectories
# Implement acceleration/deceleration curves
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
- Add obstacle detection and avoidance
- Include joint torque/force calculations  
- Add save/load functionality for robot configurations
- Implement path planning and autonomous navigation
- Add physics simulation (gravity, inertia, friction)

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
- [ ] Body pose controls don't break leg kinematics
- [ ] Height limits prevent impossible configurations
- [ ] Trajectory visualization matches actual movement
- [ ] No runtime errors or warnings
- [ ] Smooth 60 FPS animation performance

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