import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

"""
Hexapod Robot Simulator
Controls:
- W: Walk forward
- S: Walk backward  
- A: Crab walk left
- D: Crab walk right
- Q: Turn left
- E: Turn right
- Z: Move to idle position
- C: Sweep coxa joints
- G: Sweep femur joints
- T: Sweep tibia joints
- H: Raise body
- B: Lower body
"""

# ---------------- Hexapod Robot Definition ----------------
class HexapodRobot:
    def __init__(self):
        self.body_length = 2.0
        self.body_width = 1.0
        self.body_height = 0.3
        self.num_legs_per_side = 3
        self.num_legs = 6

        self.coxa_length = 0.5
        self.femur_length = 0.7
        self.tibia_length = 1.0

        self.step_period = 7

        x_offsets = np.linspace(-self.body_length/2 + 0.2, self.body_length/2 - 0.2, self.num_legs_per_side)
        y_offset_outer = self.body_width / 2
        y_offset_middle = y_offset_outer + 0.5

        self.leg_bases = [
            (x_offsets[2], -y_offset_outer, 0),  # z will be set dynamically
            (x_offsets[2],  y_offset_outer, 0),
            (x_offsets[1], -y_offset_middle, 0),
            (x_offsets[1],  y_offset_middle, 0),
            (x_offsets[0], -y_offset_outer, 0),
            (x_offsets[0],  y_offset_outer, 0),  # <-- fix here!
        ]

        angle_offset = np.deg2rad(15)
        self.leg_angles = [
            -np.pi/2 + angle_offset,
             np.pi/2 - angle_offset,
            -np.pi/2,
             np.pi/2,
            -np.pi/2 - angle_offset,
             np.pi/2 + angle_offset,
        ]

        # Tripod gait: [0, 3, 4] and [1, 2, 5] alternate
        self.tripod_phase = [0, np.pi, np.pi, 0, 0, np.pi]

    def leg_forward_kinematics(self, base, coxa_angle, femur_angle, tibia_angle):
        coxa_end = (
            base[0] + self.coxa_length * np.cos(coxa_angle),
            base[1] + self.coxa_length * np.sin(coxa_angle),
            base[2]
        )
        femur_end = (
            coxa_end[0] + self.femur_length * np.cos(coxa_angle) * np.cos(femur_angle),
            coxa_end[1] + self.femur_length * np.sin(coxa_angle) * np.cos(femur_angle),
            coxa_end[2] + self.femur_length * np.sin(femur_angle)
        )
        total_femur = femur_angle + tibia_angle
        tibia_end = (
            femur_end[0] + self.tibia_length * np.cos(coxa_angle) * np.cos(total_femur),
            femur_end[1] + self.tibia_length * np.sin(coxa_angle) * np.cos(total_femur),
            femur_end[2] + self.tibia_length * np.sin(total_femur)
        )
        return [base, coxa_end, femur_end, tibia_end]

# ---------------- 3D Inverse Kinematics ----------------
def leg_inverse_kinematics(base, foot, coxa_length, femur_length, tibia_length, coxa_mount_angle):
    # Vector from base to foot
    dx = foot[0] - base[0]
    dy = foot[1] - base[1]
    dz = foot[2] - base[2]

    # Coxa angle: angle in XY plane, relative to mounting orientation
    coxa_angle = np.arctan2(dy, dx) - coxa_mount_angle

    # Project foot position into coxa frame
    r = np.hypot(dx, dy) - coxa_length
    z = dz

    # Distance from coxa end to foot
    D = np.hypot(r, z)
    D = np.clip(D, 0.01, femur_length + tibia_length - 0.01)

    # Law of cosines for tibia
    cos_tibia = (femur_length**2 + tibia_length**2 - D**2) / (2 * femur_length * tibia_length)
    cos_tibia = np.clip(cos_tibia, -1.0, 1.0)
    tibia_angle = np.arccos(cos_tibia) - np.pi  # negative, so tibia points down from femur

    # Law of cosines for femur
    cos_femur = (D**2 + femur_length**2 - tibia_length**2) / (2 * D * femur_length)
    cos_femur = np.clip(cos_femur, -1.0, 1.0)
    femur_angle = np.arctan2(z, r) + np.arccos(cos_femur)  # zero is straight down

    return coxa_angle, femur_angle, tibia_angle

# ---------------- Handler and Animation ----------------
# Movement control variables - now additive components
move_forward = [False]
move_backward = [False]
move_left = [False]
move_right = [False]
turn_left = [False]
turn_right = [False]
sweep_coxa = [False]
sweep_femur = [False]
sweep_tibia = [False]
move_to_idle = [False]
height_limit_warned = [False]

# Camera/view control variables
camera_elevation = [30]  # Default elevation angle
camera_azimuth = [45]    # Default azimuth angle

def on_key_press(event):
    if event.key == 'w':
        move_forward[0] = True
    if event.key == 's':
        move_backward[0] = True
    if event.key == 'a':
        move_left[0] = True
    if event.key == 'd':
        move_right[0] = True
    if event.key == 'q':
        turn_left[0] = True
    if event.key == 'e':
        turn_right[0] = True
    if event.key == 'c':
        sweep_coxa[0] = True
    if event.key == 'g':
        sweep_femur[0] = True
    if event.key == 't':
        sweep_tibia[0] = True
    if event.key == 'z':
        move_to_idle[0] = True
    if event.key == 'h':
        # Calculate maximum safe height based on actual leg geometry
        # In neutral stance, legs reach horizontally: coxa + 0.8*femur
        # Maximum downward reach from that point: remaining femur + full tibia
        neutral_radius = robot.coxa_length + robot.femur_length * 0.8
        remaining_femur = robot.femur_length * 0.2  # unused femur length in neutral
        
        # Maximum vertical reach = remaining femur + full tibia (both pointing down)
        max_vertical_reach = remaining_femur + robot.tibia_length
        safety_margin = 0.2  # Conservative safety margin
        max_safe_height = max_vertical_reach - safety_margin
        
        new_height = body_z[0] + 0.05
        if new_height > max_safe_height and not height_limit_warned[0]:
            print(f"Height limit reached! Max safe height: {max_safe_height:.2f} units")
            print(f"(Max vertical reach: {max_vertical_reach:.2f} - safety: {safety_margin:.2f})")
            height_limit_warned[0] = True
        elif new_height <= max_safe_height:
            height_limit_warned[0] = False  # Reset warning when below limit
        body_z[0] = min(new_height, max_safe_height)  # raise body with height limit
    if event.key == 'b':
        body_z[0] = max(0.05, body_z[0] - 0.05)  # lower body, min height 0.05
    
    # Robot body pose controls
    if event.key == 'i':
        body_pitch[0] = min(30, body_pitch[0] + 2)  # pitch up (nose up, max 30°)
    if event.key == 'k':
        body_pitch[0] = max(-30, body_pitch[0] - 2)  # pitch down (nose down, min -30°)
    if event.key == 'j':
        body_yaw[0] = (body_yaw[0] - 2) % 360  # yaw left
    if event.key == 'l':
        body_yaw[0] = (body_yaw[0] + 2) % 360  # yaw right
    if event.key == 'm':
        body_pitch[0] = 0.0  # reset pose to neutral
        body_roll[0] = 0.0
        body_yaw[0] = 0.0

def on_key_release(event):
    if event.key == 'w':
        move_forward[0] = False
    if event.key == 's':
        move_backward[0] = False
    if event.key == 'a':
        move_left[0] = False
    if event.key == 'd':
        move_right[0] = False
    if event.key == 'q':
        turn_left[0] = False
    if event.key == 'e':
        turn_right[0] = False
    if event.key == 'c':
        sweep_coxa[0] = False
    if event.key == 'g':
        sweep_femur[0] = False
    if event.key == 't':
        sweep_tibia[0] = False
    if event.key == 'z':
        move_to_idle[0] = False

# ---------------- Visualization Setup ----------------
robot = HexapodRobot()
body_z = [robot.body_height / 2]  # Start with body at half its thickness above ground
num_legs = robot.num_legs

# Robot body pose control variables
body_pitch = [0.0]  # Tilt forward/backward (degrees)
body_roll = [0.0]   # Tilt left/right (degrees)
body_yaw = [0.0]    # Rotate left/right (degrees)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([2,2,1.0])
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(0, 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

for i, base in enumerate(robot.leg_bases):
    ax.text(base[0], base[1], base[2] + 0.1, f"{i}", color='black', fontsize=10, ha='center', va='bottom')

coxa_legend, = ax.plot([], [], [], 'o-', lw=3, color='b', label='Coxa')
femur_legend, = ax.plot([], [], [], 'o-', lw=3, color='g', label='Femur')
tibia_legend, = ax.plot([], [], [], 'o-', lw=3, color='r', label='Tibia')
ax.legend(loc='upper left')

leg_lines = []
for _ in range(num_legs):
    leg_lines.append([ax.plot([], [], [], 'o-', lw=3, color='b')[0],
                      ax.plot([], [], [], 'o-', lw=3, color='g')[0],
                      ax.plot([], [], [], 'o-', lw=3, color='r')[0]])

def init():
    for segments in leg_lines:
        for seg in segments:
            seg.set_data([], [])
            seg.set_3d_properties([])
    return [seg for segments in leg_lines for seg in segments]

# Store actual foot positions for trajectory calculation
current_foot_positions = [None for _ in range(num_legs)]

# Store target foot positions in world coordinates (before body rotation)
target_foot_positions = [None for _ in range(num_legs)]

# Initialize target foot positions for idle stance
def initialize_target_feet():
    idle_angles_deg = [75, 75, 90, 90, 105, 105]
    idle_angles_rad = [np.deg2rad(a) for a in idle_angles_deg]
    foot_radius = robot.coxa_length + robot.femur_length * 0.8
    
    for i in range(num_legs):
        sign = -1 if i % 2 == 0 else 1
        angle = sign * idle_angles_rad[i]
        # Calculate foot position relative to original (unrotated) body center
        foot_x = robot.leg_bases[i][0] + foot_radius * np.cos(angle)
        foot_y = robot.leg_bases[i][1] + foot_radius * np.sin(angle)
        foot_z = 0  # always on ground
        target_foot_positions[i] = (foot_x, foot_y, foot_z)

# Initialize the target positions
initialize_target_feet()

def animate(frame, ax=ax, robot=robot, leg_lines=leg_lines):
    # Draw updated body at current Z position first
    z_bottom = body_z[0]
    z_top = body_z[0] + robot.body_height
    
    # Apply body rotation to leg bases
    pitch_rad = np.deg2rad(body_pitch[0])
    roll_rad = np.deg2rad(body_roll[0])
    yaw_rad = np.deg2rad(body_yaw[0])
    
    # Rotation matrices
    cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
    cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
    cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
    
    # Combined rotation matrix (yaw * pitch * roll)
    R11 = cos_y * cos_p
    R12 = cos_y * sin_p * sin_r - sin_y * cos_r
    R13 = cos_y * sin_p * cos_r + sin_y * sin_r
    R21 = sin_y * cos_p
    R22 = sin_y * sin_p * sin_r + cos_y * cos_r
    R23 = sin_y * sin_p * cos_r - cos_y * sin_r
    R31 = -sin_p
    R32 = cos_p * sin_r
    R33 = cos_p * cos_r
    
    # Apply rotation to leg base positions
    leg_bases = []
    for base in robot.leg_bases:
        # Rotate relative to body center
        x, y, z = base[0], base[1], z_top
        
        # Apply rotation matrix
        new_x = R11 * x + R12 * y + R13 * z
        new_y = R21 * x + R22 * y + R23 * z
        new_z = R31 * x + R32 * y + R33 * z
        
        leg_bases.append((new_x, new_y, new_z))
    
    # No need to capture movement start positions - always use current positions

    if hasattr(ax, 'angle_texts'):
        for txt in ax.angle_texts:
            txt.remove()
    ax.angle_texts = []

    # Remove previous body box and triangle
    if hasattr(ax, 'body_artists'):
        for artist in ax.body_artists:
            artist.remove()
    ax.body_artists = []

    # Remove previous foot path lines
    if hasattr(ax, 'foot_path_lines'):
        for line in ax.foot_path_lines:
            line.remove()
    ax.foot_path_lines = []

    # Draw updated body at current Z position
    z_bottom = body_z[0]
    z_top = body_z[0] + robot.body_height
    # Create body vertices
    coffin_xy = np.array([
        [ robot.body_length/2,           0],
        [ robot.body_length/4,  robot.body_width/2],
        [-robot.body_length/4,  robot.body_width/2],
        [-robot.body_length/2,           0],
        [-robot.body_length/4, -robot.body_width/2],
        [ robot.body_length/4, -robot.body_width/2],
    ])
    
    # Apply rotation to body vertices
    top_vertices = []
    bottom_vertices = []
    
    for x, y in coffin_xy:
        # Top vertices
        xt = R11 * x + R12 * y + R13 * z_top
        yt = R21 * x + R22 * y + R23 * z_top
        zt = R31 * x + R32 * y + R33 * z_top
        top_vertices.append([xt, yt, zt])
        
        # Bottom vertices  
        xb = R11 * x + R12 * y + R13 * z_bottom
        yb = R21 * x + R22 * y + R23 * z_bottom
        zb = R31 * x + R32 * y + R33 * z_bottom
        bottom_vertices.append([xb, yb, zb])
    
    top = np.array(top_vertices)
    bottom = np.array(bottom_vertices)
    
    faces = [bottom, top]
    for i in range(6):
        j = (i + 1) % 6
        face = np.array([bottom[i], bottom[j], top[j], top[i]])
        faces.append(face)
    box = Poly3DCollection(faces, facecolors='gray', linewidths=1, edgecolors='k', alpha=0.4)
    ax.add_collection3d(box)
    
    # Rotated triangle (front indicator)
    triangle_verts = [
        [robot.body_length/2, 0, z_top],
        [robot.body_length/2 - 0.2,  robot.body_width/4, z_top],
        [robot.body_length/2 - 0.2, -robot.body_width/4, z_top],
    ]
    rotated_triangle = []
    for x, y, z in triangle_verts:
        xt = R11 * x + R12 * y + R13 * z
        yt = R21 * x + R22 * y + R23 * z
        zt = R31 * x + R32 * y + R33 * z
        rotated_triangle.append([xt, yt, zt])
    
    triangle = np.array(rotated_triangle)
    tri_artist = Poly3DCollection([triangle], facecolors='orange', edgecolors='k', alpha=0.8)
    ax.add_collection3d(tri_artist)
    ax.body_artists = [box, tri_artist]

    x_disp = 5.0
    y_disp = 0.0
    z_start = 10.0
    dz = -1.0

    # --- Visualize foot paths if any movement is active ---
    foot_path_lines = []
    any_movement = move_forward[0] or move_backward[0] or move_left[0] or move_right[0] or turn_left[0] or turn_right[0]
    if any_movement:
        num_points = 100
        
        # First, we need to calculate current joint angles for all legs to get current foot positions
        current_leg_states = []
        for i in range(num_legs):
            base_x, base_y, base_z = leg_bases[i]
            foot_x = base_x
            foot_y = base_y
            foot_z = 0

            # Calculate current joint angles based on current movement state
            if sweep_coxa[0]:
                sweep_deg = 90 * np.sin(2 * np.pi * (frame % robot.step_period) / robot.step_period)
                sweep_rad = np.deg2rad(sweep_deg)
                coxa_angle = robot.leg_angles[i] + sweep_rad
                femur_angle = 0
                tibia_angle = 0
            elif any_movement:
                # Use the same calculation as main animation for consistency
                phase = robot.tripod_phase[i]
                gait_phase = 2 * np.pi * (frame / robot.step_period) + phase
                stride_amplitude = 0.5
                crab_amplitude = 0.3
                turn_amplitude = 0.2
                lift_height = 0.3
                
                # Calculate neutral position in world coordinates (before rotation)
                neutral_radius = robot.coxa_length + robot.femur_length * 0.8
                idle_angles_deg = [75, 75, 90, 90, 105, 105]
                idle_angles_rad = [np.deg2rad(a) for a in idle_angles_deg]
                sign = -1 if i % 2 == 0 else 1
                angle = sign * idle_angles_rad[i]
                neutral_x = robot.leg_bases[i][0] + neutral_radius * np.cos(angle)  # Use original leg base
                neutral_y = robot.leg_bases[i][1] + neutral_radius * np.sin(angle)
                
                # Combine movement components additively
                stride_x = 0
                stride_y = 0
                turn_component = 0
                
                # Forward/backward movement
                if move_forward[0]:
                    stride_x += stride_amplitude * np.sin(gait_phase)
                if move_backward[0]:
                    stride_x -= stride_amplitude * np.sin(gait_phase)
                
                # Left/right movement (crab walking)
                if move_left[0]:
                    stride_y += crab_amplitude * np.sin(gait_phase)
                if move_right[0]:
                    stride_y -= crab_amplitude * np.sin(gait_phase)
                
                # Turning (modifies the base angle)
                if turn_left[0]:
                    turn_component += turn_amplitude * np.sin(gait_phase)
                if turn_right[0]:
                    turn_component -= turn_amplitude * np.sin(gait_phase)
                
                # Calculate foot position in world coordinates
                foot_x = neutral_x + stride_x
                foot_y = neutral_y + stride_y
                
                # Add turning by rotating around world origin
                if turn_component != 0:
                    rel_x = foot_x
                    rel_y = foot_y
                    cos_turn = np.cos(turn_component)
                    sin_turn = np.sin(turn_component)
                    new_rel_x = rel_x * cos_turn - rel_y * sin_turn
                    new_rel_y = rel_x * sin_turn + rel_y * cos_turn
                    foot_x = new_rel_x
                    foot_y = new_rel_y
                
                # Lift foot during swing phase
                if np.sin(gait_phase) > 0:
                    foot_z = lift_height * np.sin(gait_phase)
                else:
                    foot_z = 0
                    
                coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                    leg_bases[i], (foot_x, foot_y, foot_z),
                    robot.coxa_length, robot.femur_length, robot.tibia_length,
                    robot.leg_angles[i]
                )
            else:
                # Idle position
                idle_angles_deg = [75, 75, 90, 90, 105, 105]
                idle_angles_rad = [np.deg2rad(a) for a in idle_angles_deg]
                sign = -1 if i % 2 == 0 else 1
                angle = sign * idle_angles_rad[i]
                foot_radius = robot.coxa_length + robot.femur_length * 0.8
                foot_x = leg_bases[i][0] + foot_radius * np.cos(angle)
                foot_y = leg_bases[i][1] + foot_radius * np.sin(angle)
                foot_z = 0
                
                coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                    leg_bases[i], (foot_x, foot_y, foot_z),
                    robot.coxa_length, robot.femur_length, robot.tibia_length,
                    robot.leg_angles[i]
                )
            
            # Get actual current foot position from forward kinematics
            joints = robot.leg_forward_kinematics(leg_bases[i], coxa_angle + robot.leg_angles[i], femur_angle, tibia_angle)
            current_foot_pos = joints[3]
            current_leg_states.append(current_foot_pos)


        # Now calculate trajectories starting from actual current positions
        for i in range(num_legs):
            phase = robot.tripod_phase[i]
            gait_phases = np.linspace(0, 2*np.pi, num_points)
            start_x, start_y, start_z = current_leg_states[i]


            # Additive trajectory calculation
            stride_amplitude = 0.5
            crab_amplitude = 0.3
            turn_amplitude = 0.2
            lift_height = 0.3
            
            # Calculate combined movement trajectory
            step_x = np.full(num_points, start_x)
            step_y = np.full(num_points, start_y)
            step_z = np.zeros(num_points)
            
            # Add forward/backward movement
            if move_forward[0]:
                step_x += stride_amplitude * np.sin(gait_phases)
            if move_backward[0]:
                step_x -= stride_amplitude * np.sin(gait_phases)
            
            # Add left/right movement (crab walking)
            if move_left[0]:
                step_y += crab_amplitude * np.sin(gait_phases)
            if move_right[0]:
                step_y -= crab_amplitude * np.sin(gait_phases)
            
            # Add turning movement
            if turn_left[0] or turn_right[0]:
                turn_factor = 0
                if turn_left[0]:
                    turn_factor += turn_amplitude
                if turn_right[0]:
                    turn_factor -= turn_amplitude
                
                # Apply rotation to each point in trajectory
                for j in range(num_points):
                    # Get position relative to leg base
                    rel_x = step_x[j] - leg_bases[i][0]
                    rel_y = step_y[j] - leg_bases[i][1]
                    # Apply rotation
                    turn_angle = turn_factor * np.sin(gait_phases[j])
                    cos_turn = np.cos(turn_angle)
                    sin_turn = np.sin(turn_angle)
                    new_rel_x = rel_x * cos_turn - rel_y * sin_turn
                    new_rel_y = rel_x * sin_turn + rel_y * cos_turn
                    step_x[j] = leg_bases[i][0] + new_rel_x
                    step_y[j] = leg_bases[i][1] + new_rel_y
            
            # Add lift trajectory (always present during any movement)
            step_z = np.maximum(0, lift_height * np.sin(gait_phases))
            
            path_x, path_y, path_z = step_x, step_y, step_z

            # Draw the path as a thin purple line
            line, = ax.plot(path_x, path_y, path_z, color='purple', linewidth=1, alpha=0.7)
            foot_path_lines.append(line)
        ax.foot_path_lines = foot_path_lines

    # Check if any movement is active for main animation
    any_movement = move_forward[0] or move_backward[0] or move_left[0] or move_right[0] or turn_left[0] or turn_right[0]

    for i in range(num_legs):
        base_x, base_y, base_z = leg_bases[i]
        foot_x = base_x
        foot_y = base_y
        foot_z = 0

        if sweep_coxa[0]:
            sweep_deg = 90 * np.sin(2 * np.pi * (frame % robot.step_period) / robot.step_period)
            sweep_rad = np.deg2rad(sweep_deg)
            coxa_angle = robot.leg_angles[i] + sweep_rad
            femur_angle = 0
            tibia_angle = 0
        elif sweep_femur[0]:
            sweep_deg = 90 * np.sin(2 * np.pi * (frame % robot.step_period) / robot.step_period)
            sweep_rad = np.deg2rad(sweep_deg)
            coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                leg_bases[i],
                (foot_x, foot_y, foot_z),
                robot.coxa_length,
                robot.femur_length,
                robot.tibia_length,
                robot.leg_angles[i]
            )
            femur_angle = sweep_rad
            tibia_angle = 0
        elif sweep_tibia[0]:
            sweep_deg = 90 * np.sin(2 * np.pi * (frame % robot.step_period) / robot.step_period)
            sweep_rad = np.deg2rad(sweep_deg)
            coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                leg_bases[i],
                (foot_x, foot_y, foot_z),
                robot.coxa_length,
                robot.femur_length,
                robot.tibia_length,
                robot.leg_angles[i]
            )
            femur_angle = 0
            tibia_angle = sweep_rad
        elif move_to_idle[0]:
            # Move legs to idle position - neutral stance with predefined angles
            # Calculate idle foot position based on current body height
            neutral_radius = robot.coxa_length + robot.femur_length * 0.8
            idle_angles_deg = [75, 75, 90, 90, 105, 105]  # Predefined idle angles for each leg
            idle_angles_rad = [np.deg2rad(a) for a in idle_angles_deg]
            sign = -1 if i % 2 == 0 else 1  # Right legs negative, left legs positive
            angle = sign * idle_angles_rad[i]
            
            # Calculate idle position in world coordinates (before body rotation)
            idle_x = robot.leg_bases[i][0] + neutral_radius * np.cos(angle)
            idle_y = robot.leg_bases[i][1] + neutral_radius * np.sin(angle)
            idle_z = 0  # Keep feet on ground
            
            # Store target position for inverse kinematics
            target_foot_positions[i] = (idle_x, idle_y, idle_z)
            
            # Use inverse kinematics to reach idle position
            coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                leg_bases[i],
                target_foot_positions[i],
                robot.coxa_length,
                robot.femur_length,
                robot.tibia_length,
                robot.leg_angles[i]
            )
        elif any_movement:
            # Calculate proper trajectory positions for movement
            phase = robot.tripod_phase[i]
            gait_phase = 2 * np.pi * (frame / robot.step_period) + phase
            stride_amplitude = 0.5
            crab_amplitude = 0.3
            turn_amplitude = 0.2
            lift_height = 0.3
            
            # Calculate neutral position in unrotated world coordinates
            neutral_radius = robot.coxa_length + robot.femur_length * 0.8
            idle_angles_deg = [75, 75, 90, 90, 105, 105]
            idle_angles_rad = [np.deg2rad(a) for a in idle_angles_deg]
            sign = -1 if i % 2 == 0 else 1
            angle = sign * idle_angles_rad[i]
            neutral_x = robot.leg_bases[i][0] + neutral_radius * np.cos(angle)  # Use original leg base
            neutral_y = robot.leg_bases[i][1] + neutral_radius * np.sin(angle)
            
            # Combine movement components additively
            stride_x = 0
            stride_y = 0
            turn_component = 0
            
            # Forward/backward movement
            if move_forward[0]:
                stride_x += stride_amplitude * np.sin(gait_phase)
            if move_backward[0]:
                stride_x -= stride_amplitude * np.sin(gait_phase)
            
            # Left/right movement (crab walking)
            if move_left[0]:
                stride_y += crab_amplitude * np.sin(gait_phase)
            if move_right[0]:
                stride_y -= crab_amplitude * np.sin(gait_phase)
            
            # Turning (modifies the base angle)
            if turn_left[0]:
                turn_component += turn_amplitude * np.sin(gait_phase)
            if turn_right[0]:
                turn_component -= turn_amplitude * np.sin(gait_phase)
            
            # Calculate foot position in world coordinates (before body rotation)
            foot_x = neutral_x + stride_x
            foot_y = neutral_y + stride_y
            
            # Add turning by rotating the foot position around original body center
            if turn_component != 0:
                # Calculate foot position relative to original body center
                rel_x = foot_x
                rel_y = foot_y
                # Apply rotation
                cos_turn = np.cos(turn_component)
                sin_turn = np.sin(turn_component)
                new_rel_x = rel_x * cos_turn - rel_y * sin_turn
                new_rel_y = rel_x * sin_turn + rel_y * cos_turn
                foot_x = new_rel_x
                foot_y = new_rel_y
            
            # Update target position for this movement
            target_foot_positions[i] = (foot_x, foot_y, 0)
            
            # Lift foot during swing phase
            if np.sin(gait_phase) > 0:
                foot_z = lift_height * np.sin(gait_phase)
            else:
                foot_z = 0
                
            coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                leg_bases[i],
                (foot_x, foot_y, foot_z),
                robot.coxa_length,
                robot.femur_length,
                robot.tibia_length,
                robot.leg_angles[i]
            )




        else:
            # Use stored target foot positions (maintains ground contact during body rotation)
            foot_x, foot_y, foot_z = target_foot_positions[i]

            coxa_angle, femur_angle, tibia_angle = leg_inverse_kinematics(
                leg_bases[i],
                (foot_x, foot_y, foot_z),
                robot.coxa_length,
                robot.femur_length,
                robot.tibia_length,
                robot.leg_angles[i]
            )

        coxa_deg = np.rad2deg(coxa_angle)
        femur_deg = np.rad2deg(femur_angle)
        tibia_deg = np.rad2deg(tibia_angle)
        txt_c = ax.text(x_disp, y_disp, z_start + dz * (i * 3 + 0), f"c{i}: {coxa_deg:.0f}°", color='blue', fontsize=10, ha='left', va='center')
        txt_f = ax.text(x_disp, y_disp, z_start + dz * (i * 3 + 1), f"f{i}: {femur_deg:.0f}°", color='green', fontsize=10, ha='left', va='center')
        txt_t = ax.text(x_disp, y_disp, z_start + dz * (i * 3 + 2), f"t{i}: {tibia_deg:.0f}°", color='red', fontsize=10, ha='left', va='center')
        ax.angle_texts.extend([txt_c, txt_f, txt_t])

        joints = robot.leg_forward_kinematics(leg_bases[i], coxa_angle + robot.leg_angles[i], femur_angle, tibia_angle)
        
        # Store the actual current foot position (tip of tibia from forward kinematics)
        actual_foot_pos = joints[3]  # The fourth joint is the foot (tip of tibia)
        current_foot_positions[i] = actual_foot_pos
        
        leg_lines[i][0].set_data([joints[0][0], joints[1][0]], [joints[0][1], joints[1][1]])
        leg_lines[i][0].set_3d_properties([joints[0][2], joints[1][2]])
        leg_lines[i][1].set_data([joints[1][0], joints[2][0]], [joints[1][1], joints[2][1]])
        leg_lines[i][1].set_3d_properties([joints[1][2], joints[2][2]])
        leg_lines[i][2].set_data([joints[2][0], joints[3][0]], [joints[2][1], joints[3][1]])
        leg_lines[i][2].set_3d_properties([joints[2][2], joints[3][2]])



    return [seg for segments in leg_lines for seg in segments] + foot_path_lines

ani = FuncAnimation(
    fig,
    animate,
    frames=range(robot.step_period),
    init_func=init,
    blit=False,
    repeat=True
)

# Disable default matplotlib key bindings to prevent conflicts
plt.rcParams['keymap.save'].remove('s')
plt.rcParams['keymap.save'] = []  # Clear all save shortcuts
plt.rcParams['keymap.quit'].clear()  # Clear quit shortcuts
plt.rcParams['keymap.fullscreen'].clear()  # Clear fullscreen shortcuts
plt.rcParams['keymap.home'].clear()  # Clear home shortcuts
plt.rcParams['keymap.back'].clear()  # Clear back shortcuts
plt.rcParams['keymap.forward'].clear()  # Clear forward shortcuts
plt.rcParams['keymap.pan'].clear()  # Clear pan shortcuts
plt.rcParams['keymap.zoom'].clear()  # Clear zoom shortcuts
plt.rcParams['keymap.grid'].clear()  # Clear grid shortcuts

fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)

plt.show()