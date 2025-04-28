import FreeCAD as App
import FreeCADGui as Gui
import Part
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import PySide
from PySide import QtGui, QtCore

class Quaternion:
    """Class to handle quaternion operations for 3D rotations
    
    Quaternions provide a compact and numerically stable way to represent 3D rotations,
    avoiding issues like gimbal lock that can occur with Euler angles.
    
    A quaternion is represented as q = w + xi + yj + zk, where:
    - w is the scalar (real) part
    - x, y, z form the vector (imaginary) part
    - i, j, k are imaginary units where i² = j² = k² = ijk = -1
    
    For rotation quaternions, we ensure they are unit quaternions (magnitude = 1).
    """
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w  # Scalar (real) part
        self.x = x  # First component of vector (imaginary) part
        self.y = y  # Second component of vector (imaginary) part
        self.z = z  # Third component of vector (imaginary) part
        self.normalize()  # Ensure we have a unit quaternion
    
    def from_axis_angle(self, axis, angle_deg):
        """Create quaternion from rotation axis and angle (in degrees)
        
        This implements the formula:
        q = cos(θ/2) + sin(θ/2)*(x*i + y*j + z*k)
        
        Where:
        - θ is the rotation angle
        - (x,y,z) is the normalized rotation axis
        """
        # Convert to radians
        angle_rad = np.radians(angle_deg)
        
        # Normalize axis
        if isinstance(axis, App.Vector):
            # Convert FreeCAD vector to numpy array
            axis_np = np.array([axis.x, axis.y, axis.z])
        else:
            axis_np = np.array(axis)
            
        # Handle zero vector case
        norm = np.linalg.norm(axis_np)
        if norm < 1e-10:
            return self
            
        axis_np = axis_np / norm
        
        # Calculate quaternion components
        half_angle = angle_rad / 2.0
        sin_half = np.sin(half_angle)
        
        self.w = np.cos(half_angle)     # Scalar part = cos(θ/2)
        self.x = axis_np[0] * sin_half    # Vector part = axis * sin(θ/2)
        self.y = axis_np[1] * sin_half
        self.z = axis_np[2] * sin_half
        
        return self
    
    def from_euler(self, roll_deg, pitch_deg, yaw_deg):
        """Create quaternion from Euler angles (in degrees)"""
        # Convert to radians
        roll = np.radians(roll_deg)
        pitch = np.radians(pitch_deg)
        yaw = np.radians(yaw_deg)
        
        cr = np.cos(roll / 2.0)
        sr = np.sin(roll / 2.0)
        cp = np.cos(pitch / 2.0)
        sp = np.sin(pitch / 2.0)
        cy = np.cos(yaw / 2.0)
        sy = np.sin(yaw / 2.0)
        
        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy
        
        return self
    
    def normalize(self):
        """Normalize the quaternion"""
        norm = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm > 1e-10:
            self.w /= norm
            self.x /= norm
            self.y /= norm
            self.z /= norm
        return self
    
    def conjugate(self):
        """Return the conjugate of this quaternion"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def multiply(self, q2):
        """Multiply this quaternion by another quaternion
        
        Quaternion multiplication represents composition of rotations.
        If q1 represents rotation R1 and q2 represents rotation R2,
        then q1*q2 represents applying rotation R2 followed by R1.
        
        The quaternion multiplication formula is:
        (a,b)*(c,d) = (ac-b·d, ad+cb+b×d)
        where a,c are scalar parts, b,d are vector parts,
        · is dot product, and × is cross product.
        """
        # Extract components for clarity
        a1, b1 = self.w, np.array([self.x, self.y, self.z])
        a2, b2 = q2.w, np.array([q2.x, q2.y, q2.z])
        
        # Calculate new scalar part: a1*a2 - b1·b2 (dot product)
        w = a1*a2 - np.dot(b1, b2)
        
        # Calculate new vector part: a1*b2 + a2*b1 + b1×b2 (cross product)
        # We expand this manually for educational purposes
        x = a1*b2[0] + a2*b1[0] + (b1[1]*b2[2] - b1[2]*b2[1])
        y = a1*b2[1] + a2*b1[1] + (b1[2]*b2[0] - b1[0]*b2[2])
        z = a1*b2[2] + a2*b1[2] + (b1[0]*b2[1] - b1[1]*b2[0])
        
        return Quaternion(w, x, y, z)
    
    def rotate_vector(self, v):
        """Rotate a vector by this quaternion"""
        # Always return a FreeCAD vector
        # First convert to numpy array for calculation
        if isinstance(v, App.Vector):
            vector = np.array([v.x, v.y, v.z])
        else:
            vector = np.array(v)
            
        # Convert vector to quaternion with w=0
        v_quat = Quaternion(0, vector[0], vector[1], vector[2])
        
        # Perform rotation: q * v_quat * q^(-1)
        q_conj = self.conjugate()
        rotated = self.multiply(v_quat).multiply(q_conj)
        
        # Always return FreeCAD vector for consistency
        return App.Vector(rotated.x, rotated.y, rotated.z)
    
    def to_rotation_matrix(self):
        """Convert quaternion to 3x3 rotation matrix"""
        xx = self.x * self.x
        xy = self.x * self.y
        xz = self.x * self.z
        xw = self.x * self.w
        yy = self.y * self.y
        yz = self.y * self.z
        yw = self.y * self.w
        zz = self.z * self.z
        zw = self.z * self.w
        
        matrix = np.zeros((3, 3))
        matrix[0, 0] = 1 - 2 * (yy + zz)
        matrix[0, 1] = 2 * (xy - zw)
        matrix[0, 2] = 2 * (xz + yw)
        matrix[1, 0] = 2 * (xy + zw)
        matrix[1, 1] = 1 - 2 * (xx + zz)
        matrix[1, 2] = 2 * (yz - xw)
        matrix[2, 0] = 2 * (xz - yw)
        matrix[2, 1] = 2 * (yz + xw)
        matrix[2, 2] = 1 - 2 * (xx + yy)
        
        return matrix
    
    def to_freecad_rotation(self):
        """Convert quaternion to FreeCAD Rotation"""
        # FreeCAD stores rotation as quaternion internally
        return App.Rotation(self.x, self.y, self.z, self.w)
    
    @staticmethod
    def from_freecad_rotation(rotation):
        """Create quaternion from FreeCAD Rotation"""
        q = rotation.Q
        # Note: FreeCAD stores quaternions as (x, y, z, w) but we use (w, x, y, z)
        return Quaternion(q[3], q[0], q[1], q[2])
    
    def to_axis_angle(self):
        """Convert quaternion to axis-angle representation"""
        # Check if the rotation is close to identity
        if abs(self.w) > 0.9999:
            return np.array([1.0, 0.0, 0.0]), 0.0
            
        angle = 2 * np.acos(self.w)
        s = np.sqrt(1 - self.w * self.w)
        
        if s < 1e-10:  # To avoid division by zero
            return np.array([1.0, 0.0, 0.0]), 0.0
            
        axis = np.array([self.x / s, self.y / s, self.z / s])
        return axis, angle

def create_rounded_link(length, width, height, position, name, color):
    """Create a link with rounded edges"""
    # Create base box
    # For base, center it on both length and width
    if name == "Base":
        box = Part.makeBox(length, width, height, 
                          App.Vector(-length/2, -width/2, 0))  # Center on both axes
    else:
        # CRITICAL FIX: For links, don't center on length - we want links to extend along X-axis
        # This is crucial for the robot arm to visualize correctly
        # The origin needs to be at one end of the link, and the link should extend along X
        box = Part.makeBox(length, width, height, 
                          App.Vector(0, -width/2, -height/2))
    
    # Add fillets to all edges
    radius = min(width, height) / 4
    edges = box.Edges
    box_filleted = box.makeFillet(radius, edges)
    
    # Create a FreeCAD object from the shape
    link = App.ActiveDocument.addObject("Part::Feature", name)
    link.Shape = box_filleted
    link.Placement.Base = position
    
    # Set color
    if Gui.ActiveDocument:
        link_view = Gui.ActiveDocument.getObject(name)
        link_view.ShapeColor = color
    
    return link

def create_joint(position, radius, height, name, color):
    """Create a spherical joint
    
    Parameters:
    - position: 3D position vector for the joint center
    - radius: Radius of the joint sphere
    - height: Not used for spheres, kept for API compatibility
    - name: Object name in FreeCAD
    - color: RGB tuple for joint color
    
    Returns:
    - FreeCAD object representing the joint
    """
    # Create a sphere centered at origin
    # Note: For a sphere, we only need the radius parameter
    sphere = Part.makeSphere(radius)
    
    # Create FreeCAD object
    joint = App.ActiveDocument.addObject("Part::Feature", name)
    joint.Shape = sphere
    joint.Placement.Base = position
    
    # Set color
    if Gui.ActiveDocument:
        joint_view = Gui.ActiveDocument.getObject(name)
        joint_view.ShapeColor = color
    
    return joint

class RoboticArm:
    """Class to handle a robotic arm with quaternions and Jacobian-based kinematics
    
    This class demonstrates:
    1. Forward kinematics using quaternion rotations
    2. Jacobian matrix calculation for velocity mapping
    3. 3D visualization in FreeCAD with proper link chain representation
    
    The robot has a fixed base and 3 revolute joints with links between them.
    Joint angles are specified in degrees, with zero angles corresponding to
    all links extended horizontally along the X-axis.
    """
    
    def __init__(self):
        # Create a new document
        self.doc = App.newDocument("RoboticArm3D")
        
        # Base dimensions
        self.base_length = 60
        self.base_width = 60
        self.base_height = 20

        # Joint dimensions
        self.joint_radius = 10  # Increased radius for better visibility as spheres
        self.joint_height = 10  # Kept for compatibility, not used with spheres

        # Link dimensions
        self.link_width = self.joint_radius * 1.5  # Adjusted width proportional to joint radius
        self.link_height = self.joint_height
        self.link_lengths = [80, 60, 40]  # Length of each link
        
        # Colors (RGB tuples)
        self.base_color = (0.2, 0.2, 0.2)  # Dark gray
        self.joint_color = (0.7, 0.1, 0.1)  # Red for better contrast with links
        self.link_colors = [
            (0.0, 0.4, 0.8),  # Blue
            (0.0, 0.6, 0.8),  # Light blue
            (0.0, 0.8, 0.8)   # Cyan
        ]
        
        # Joint information
        self.num_joints = 3
        self.joint_axes = [
            App.Vector(0, 0, 1),  # Joint 1: rotation about Z
            App.Vector(0, 1, 0),  # Joint 2: rotation about Y
            App.Vector(1, 0, 0)   # Joint 3: rotation about X
        ]
        
        # Current state
        self.joint_angles = np.zeros(self.num_joints)  # In degrees
        self.joint_positions = []
        self.joint_orientations = []
        
        # Initialize positions and orientations
        for _ in range(self.num_joints + 1):  # +1 for end-effector
            self.joint_positions.append(App.Vector(0, 0, 0))
            self.joint_orientations.append(Quaternion())
            
        # Objects
        self.base = None
        self.joints = []
        self.links = []
        
    def create_robot(self):
        """Create the robot arm in FreeCAD"""
        # Set up graph paper background for the view if GUI is available
        if Gui.ActiveDocument:
            try:
                # Set background to a light blue-gray color (common for graph paper)
                bg_color = (0.97, 0.97, 0.99)  # Very light blue-gray
                
                # Get the current view and set its properties
                view = Gui.ActiveDocument.ActiveView
                
                # Set background color
                view.setBackgroundColor(*bg_color)
                
                # Add a graph paper grid
                # Enable grid
                param = App.ParamGet("User parameter:BaseApp/Preferences/View")
                param.SetBool("ShowGrid", True)
                
                # Set grid to be visible
                view.setAxisCross(True)  # Show coordinate axis
                
                # Configure grid appearance
                grid_settings = Gui.ActiveDocument.ActiveView.getViewer().getGridSettings()
                grid_settings[0] = 10.0  # Grid size units
                grid_settings[1] = 10    # Number of subdivisions
                grid_settings[2] = (0.8, 0.8, 0.8)  # Main grid color (light gray)
                grid_settings[3] = (0.9, 0.9, 0.9)  # Subdivision color (lighter gray)
                
                # Apply grid settings
                Gui.ActiveDocument.ActiveView.getViewer().setGridSettings(grid_settings)
                
                # Make sure the grid is shown
                Gui.ActiveDocument.ActiveView.getViewer().getViewport()
                
                print("Graph paper background applied successfully")
            except Exception as e:
                print(f"Could not set graph paper background: {e}")
                
        # Create base at origin
        self.base = create_rounded_link(
            self.base_length, self.base_width, self.base_height,
            App.Vector(0, 0, 0), "Base", self.base_color
        )
        
        # Create a fixed vertical bar to raise the first joint
        bar_height = 40  # Height of the vertical bar
        bar_width = self.joint_radius * 1.5  # Width of the bar
        
        # Create a vertical support bar
        support_bar = Part.makeBox(
            bar_width, bar_width, bar_height,
            App.Vector(-bar_width/2, -bar_width/2, self.base_height)
        )
        
        # Add fillets to the vertical bar edges
        radius = bar_width / 4
        edges = support_bar.Edges
        bar_filleted = support_bar.makeFillet(radius, edges)
        
        # Create a FreeCAD object from the shape
        self.support = App.ActiveDocument.addObject("Part::Feature", "Support")
        self.support.Shape = bar_filleted
        
        # Set color - same as base for continuity
        if Gui.ActiveDocument:
            support_view = Gui.ActiveDocument.getObject("Support")
            support_view.ShapeColor = self.base_color
        
        # Initial base joint position (centered on top of the support bar)
        # This is now elevated by the support bar height
        self.joint_positions[0] = App.Vector(0, 0, self.base_height + bar_height)
        
        # Create all joints
        for i in range(self.num_joints):
            joint = create_joint(
                self.joint_positions[0],  # Will be updated later
                self.joint_radius, self.joint_height,
                f"Joint{i+1}", self.joint_color
            )
            self.joints.append(joint)
        
        # Create links
        for i, (length, color) in enumerate(zip(self.link_lengths, self.link_colors)):
            link = create_rounded_link(
                length, self.link_width, self.link_height,
                self.joint_positions[0],  # Will be updated later
                f"Link{i+1}", color
            )
            self.links.append(link)
        
        # Update the robot with initial zero angles
        self.update_kinematics()
        self.update_freecad_objects()
        
        # Set up proper view with better zoom
        if Gui.ActiveDocument:
            # Set to isometric view
            Gui.ActiveDocument.ActiveView.viewIsometric()
            # Ensure the camera is pointing slightly downward
            Gui.ActiveDocument.ActiveView.setCameraType("Perspective")
            
            # First fit view to see all objects
            Gui.SendMsgToActiveView("ViewFit")
            
            # Add a small delay to ensure view updates
            time.sleep(0.5)
            
            # Get the camera and adjust zoom
            cam = Gui.ActiveDocument.ActiveView.getCameraNode()
            
            # If we can access the camera position directly, adjust it
            try:
                # Get current camera position
                pos = cam.position.getValue()
                # Move camera back by scaling the position vector (zoom out)
                zoom_factor = 1.5  # Increase this number to zoom out more
                new_pos = (pos[0] * zoom_factor, pos[1] * zoom_factor, pos[2] * zoom_factor)
                # Set new camera position
                cam.position.setValue(new_pos)
            except:
                # Alternative method if direct camera manipulation fails
                # Use ViewFit and then attempt to zoom out
                Gui.SendMsgToActiveView("ViewFit")
                # Try to zoom out with commands
                for i in range(5):  # Zoom out 5 steps
                    Gui.SendMsgToActiveView("ZoomOut")
            
            # Fit again to ensure everything is visible but zoomed out
            time.sleep(0.5)
        
        # Recompute document
        self.doc.recompute()
    
    def update_kinematics(self):
        """Update forward kinematics based on current joint angles
        
        Forward kinematics computes the position and orientation of 
        each joint and the end-effector based on joint angles.
        
        The process follows these steps:
        1. Start at the base with initial position and orientation
        2. For each joint in the chain:
           a. Store current position and orientation for this joint
           b. Apply joint rotation using quaternions
           c. Move along the link to the next joint position
        3. Finally, set the end-effector position and orientation
        
        This is an implementation of the Denavit-Hartenberg method
        simplified for a sequence of rotational joints.
        """
        # Start at the base joint position
        current_pos = App.Vector(self.joint_positions[0])
        
        # Start with identity orientation (no rotation)
        current_orientation = Quaternion()  # Identity quaternion = no rotation
        
        # Store the base joint's position and orientation
        self.joint_positions[0] = App.Vector(current_pos)
        self.joint_orientations[0] = Quaternion(
            current_orientation.w, current_orientation.x,
            current_orientation.y, current_orientation.z
        )
        
        # Process each joint in the kinematic chain
        for i in range(self.num_joints):
            # STEP 1: Apply joint rotation
            # Convert joint angle to quaternion rotation around joint axis
            joint_quat = Quaternion().from_axis_angle(self.joint_axes[i], self.joint_angles[i])
            
            # Compose with current orientation (quaternion multiplication)
            # This applies the new rotation in the local reference frame
            current_orientation = current_orientation.multiply(joint_quat)
            
            # STEP 2: Store current joint position and orientation
            self.joint_positions[i] = App.Vector(current_pos)
            self.joint_orientations[i] = Quaternion(
                current_orientation.w, current_orientation.x, 
                current_orientation.y, current_orientation.z
            )
            
            # STEP 3: Calculate position of next joint
            # First create a unit vector along X-axis (standard forward direction)
            unit_x = App.Vector(1.0, 0.0, 0.0)
            
            # Rotate this unit vector using the current orientation quaternion
            # This gives us the link direction in world coordinates
            rotated_unit = current_orientation.rotate_vector(unit_x)
            
            # Scale the rotated unit vector by the link length
            # This gives us the actual displacement from current joint to next joint
            link_length = self.link_lengths[i]
            scaled_vector = App.Vector(
                rotated_unit.x * link_length,
                rotated_unit.y * link_length,
                rotated_unit.z * link_length
            )
            
            # Update position for next joint by adding displacement vector
            current_pos = App.Vector(
                current_pos.x + scaled_vector.x,
                current_pos.y + scaled_vector.y,
                current_pos.z + scaled_vector.z
            )
            
            # Debug: Verify the link length is correctly applied
            actual_displacement = np.sqrt(scaled_vector.x**2 + scaled_vector.y**2 + scaled_vector.z**2) 
            print(f"Link {i+1} length: {link_length}, Resulting displacement: {actual_displacement:.2f}")
        
        # Finally, set end-effector position and orientation
        # The end-effector inherits the orientation of the last joint
        self.joint_positions[self.num_joints] = App.Vector(current_pos)
        self.joint_orientations[self.num_joints] = Quaternion(
            current_orientation.w, current_orientation.x,
            current_orientation.y, current_orientation.z
        )
    
    def update_freecad_objects(self):
        """Update FreeCAD objects based on current joint states"""
        # Update joints and links
        for i in range(self.num_joints):
            # Position each joint
            self.joints[i].Placement.Base = self.joint_positions[i]
            self.joints[i].Placement.Rotation = self.joint_orientations[i].to_freecad_rotation()
            
            # FIXED: Position and orient each link correctly
            # Each link starts at its corresponding joint and extends to the next joint
            self.links[i].Placement.Base = self.joint_positions[i]
            self.links[i].Placement.Rotation = self.joint_orientations[i].to_freecad_rotation()
            
            # Calculate and print distance between consecutive joints
            next_idx = i + 1
            if next_idx < len(self.joint_positions):
                curr_pos = self.joint_positions[i]
                next_pos = self.joint_positions[next_idx]
                dist = np.sqrt((next_pos.x - curr_pos.x)**2 + 
                                 (next_pos.y - curr_pos.y)**2 + 
                                 (next_pos.z - curr_pos.z)**2)
                print(f"Distance from joint {i+1} to joint/effector {next_idx+1}: {dist:.2f} units")
                
                # If link distance doesn't match expected length, print warning
                if abs(dist - self.link_lengths[i]) > 0.1:  # Allow small tolerance
                    print(f"  WARNING: Link {i+1} distance ({dist:.2f}) doesn't match length ({self.link_lengths[i]})!")
        
        # Recompute the document to reflect changes
        self.doc.recompute()
    
    def calculate_jacobian(self):
        """Calculate the Jacobian matrix for the current configuration
        
        The Jacobian matrix J maps joint velocities to end-effector velocities:
        [v_ee]   = J * [θ̇]
        [ω_ee]
        
        Where:
        - v_ee is linear velocity of end-effector (3x1)
        - ω_ee is angular velocity of end-effector (3x1)
        - θ̇ is joint velocity vector (num_joints x 1)
        - J is the Jacobian matrix (6 x num_joints)
        
        For a revolute joint i, the Jacobian columns are:
        J_i = [ z_i × (p_ee - p_i) ]  <- Linear velocity contribution
              [        z_i         ]  <- Angular velocity contribution
              
        Where:
        - z_i is the rotation axis of joint i in world frame
        - p_ee is the position of the end-effector
        - p_i is the position of joint i
        - × is the cross product
        """
        # Initialize a 6×n Jacobian matrix with zeros
        # First 3 rows correspond to linear velocity, last 3 to angular velocity
        J = np.zeros((6, self.num_joints))
        
        # Get end-effector position in world frame
        end_effector_pos = np.array([
            self.joint_positions[-1].x,
            self.joint_positions[-1].y,
            self.joint_positions[-1].z
        ])
        
        # Calculate each column of the Jacobian matrix
        for i in range(self.num_joints):
            # Get joint position in world frame
            joint_pos = np.array([
                self.joint_positions[i].x,
                self.joint_positions[i].y,
                self.joint_positions[i].z
            ])
            
            # Get joint rotation axis in world frame
            # First rotate the local axis to global coordinates using the current joint orientation
            axis_world = self.joint_orientations[i].rotate_vector(self.joint_axes[i])
            
            # Convert FreeCAD vector to numpy array for calculations
            axis_world_np = np.array([axis_world.x, axis_world.y, axis_world.z])
            
            # LINEAR VELOCITY COMPONENT:
            # Calculate the displacement vector from joint to end-effector
            r = end_effector_pos - joint_pos
            
            # The linear velocity component is the cross product of joint axis and displacement
            # This represents how the joint rotation affects end-effector linear motion
            J[0:3, i] = np.cross(axis_world_np, r)
            
            # ANGULAR VELOCITY COMPONENT:
            # The angular velocity component is simply the joint axis in world frame
            # This represents how the joint rotation affects end-effector angular motion
            J[3:6, i] = axis_world_np
        
        return J
    
    def demo_direct_joint_control(self, duration=10, steps=100, save_animation=False, output_folder=None):
        """Demonstrate direct joint control with smooth motion profiles
        
        This method:
        1. Verifies the initial link dimensions
        2. Moves the robot through a sequence of predefined poses
        3. Shows quaternion-based rotation and Jacobian matrix properties
        4. Creates a visualization of the end-effector path
        5. Optionally saves frames for creating an animation
        
        Parameters:
        - duration: Total time for the demo in seconds
        - steps: Total number of interpolation steps
        - save_animation: Whether to save frames for animation
        - output_folder: Where to save animation frames (if None, creates a folder)
        """
        print("Starting direct joint control demonstration with quaternions...")
        
        # Ensure grid is visible for better background during animation
        if Gui.ActiveDocument:
            try:
                # Re-apply graph paper settings to ensure it's visible
                param = App.ParamGet("User parameter:BaseApp/Preferences/View")
                param.SetBool("ShowGrid", True)
            except Exception as e:
                print(f"Could not enable grid for animation: {e}")
                
        # IMPORTANT: First create end-effector path visualization early
        # so it appears throughout the animation
        print("\nCreating end-effector path visualization...")
        # Visualize end-effector path with a trail
        path_points = []
        
        # Setup for saving animation frames
        frame_count = 0
        if save_animation:
            import os
            # Create output folder if not specified
            if output_folder is None:
                # Create a folder in the user's home directory
                home_dir = os.path.expanduser("~")
                output_folder = os.path.join(home_dir, "RobotArmAnimation")
            # Debug output
            print(f"DEBUG: Actual output folder absolute path: {os.path.abspath(output_folder)}")
                
            # Create output folder if it doesn't exist
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)
                print(f"Created output folder for animation: {output_folder}")
            print(f"Animation frames will be saved to: {output_folder}")
            
        # IMPORTANT: First verify links are properly scaled
        print("\nVerifying link dimensions before starting demo...")
        # Force a kinematics update with zero angles to see initial link distances
        self.joint_angles = np.zeros(self.num_joints)
        self.update_kinematics()
        
        # Check each link distance
        for i in range(self.num_joints):
            next_idx = i + 1
            if next_idx < len(self.joint_positions):
                p1 = self.joint_positions[i]
                p2 = self.joint_positions[next_idx]
                dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
                print(f"Initial distance from joint {i+1} to joint/effector {next_idx+1}: {dist:.2f} units (should be close to {self.link_lengths[i]} units)")
                
                # If distance is too small (compared to defined link length), we have a scaling issue
                if abs(dist - self.link_lengths[i]) > 1.0:
                    print(f"WARNING: Link {i+1} distance ({dist:.2f}) is significantly different from defined length ({self.link_lengths[i]})!")
                    print("This indicates a scaling issue in the kinematics calculations.")
        
        # Create a motion profile with multiple poses
        # Each pose is a list of joint angles in degrees
        poses = [
            [0, 0, 0],              # Home position
            [45, 0, 0],             # Rotate base joint
            [45, 30, 0],            # Bend middle joint
            [45, 30, 45],           # Rotate end joint
            [0, 45, 45],            # Return base to center, keep others
            [-45, 45, 45],          # Rotate base opposite
            [-45, 0, 0],            # Straighten arm
            [0, 0, 0]               # Return to home position
        ]
        
        # Add more complex poses to demonstrate 3D motion capabilities
        extended_poses = [
            [0, 0, 0],              # Home position
            [20, 30, 0],            # Slight rotation and bend
            [45, 45, 0],            # Diagonal position
            [45, 45, 45],           # Twist end effector
            [45, -20, 45],          # Bend in opposite direction
            [0, -45, 45],           # Center base, keep bend
            [-30, -20, 20],         # Complex pose with all joints
            [-45, 30, -30],         # Another complex pose
            [0, 0, 0]               # Return to home position
        ]
        
        # Choose which set of poses to use
        demo_poses = poses  # Can be changed to extended_poses for more complex motion
        
        # First run through all poses to collect the complete path
        # This calculates the complete path before animation starts
        print("Pre-calculating end-effector path...")
        for i in range(len(demo_poses) - 1):
            start_pose = np.array(demo_poses[i])
            end_pose = np.array(demo_poses[i+1])
            
            steps_for_segment = max(10, int(steps / (len(demo_poses) - 1)))
            
            for step in range(steps_for_segment + 1):
                t = step / steps_for_segment
                smooth_t = 0.5 - 0.5 * np.cos(t * np.pi)
                current_angles = start_pose + smooth_t * (end_pose - start_pose)
                
                # Set joint angles
                self.joint_angles = current_angles
                
                # Update kinematics
                self.update_kinematics()
                
                # Add end-effector position to path
                if step % 2 == 0:  # Add more points for smoother path
                    path_points.append(self.joint_positions[-1])
        
        # Create a compound path visualization before starting the animation
        if path_points:
            path_shape = self.create_path_visualization(path_points)
            path_obj = App.ActiveDocument.addObject("Part::Feature", "EndEffectorPath")
            path_obj.Shape = path_shape
            if Gui.ActiveDocument:
                Gui.ActiveDocument.getObject("EndEffectorPath").LineColor = (1.0, 0.0, 0.0)  # Red
                Gui.ActiveDocument.getObject("EndEffectorPath").PointColor = (1.0, 1.0, 0.0)  # Yellow
                Gui.ActiveDocument.getObject("EndEffectorPath").LineWidth = 4  # Thicker line
                Gui.ActiveDocument.getObject("EndEffectorPath").PointSize = 6  # Larger points
        
        # Update document to show the path
        App.ActiveDocument.recompute()
        Gui.updateGui()
        
        # Clear path_points for the actual animation
        path_points = []
        
        # For each pose transition in the actual animation
        for i in range(len(demo_poses) - 1):
            start_pose = np.array(demo_poses[i])
            end_pose = np.array(demo_poses[i+1])
            
            print(f"Moving from pose {i+1} to pose {i+2}...")
            
            # Interpolate between poses
            steps_for_segment = max(10, int(steps / (len(demo_poses) - 1)))
            
            for step in range(steps_for_segment + 1):
                t = step / steps_for_segment
                
                # Smooth interpolation using sine curve for acceleration/deceleration
                smooth_t = 0.5 - 0.5 * np.cos(t * np.pi)
                
                # Linear interpolation with smooth timing
                current_angles = start_pose + smooth_t * (end_pose - start_pose)
                
                # Set joint angles
                self.joint_angles = current_angles
                
                # Update kinematics
                self.update_kinematics()
                
                # Update visualization
                self.update_freecad_objects()
                
                # Add end-effector position to path
                if step % 5 == 0:  # Only add every 5th point to keep the path cleaner
                    path_points.append(self.joint_positions[-1])
                
                # Print info occasionally
                if step % 10 == 0 or step == steps_for_segment:
                    # Calculate Jacobian to demonstrate use
                    J = self.calculate_jacobian()
                    
                    # Display interesting info
                    end_pos = self.joint_positions[-1]
                    print(f"  Step {step+1}/{steps_for_segment+1}")
                    print(f"  End-effector position: ({end_pos.x:.1f}, {end_pos.y:.1f}, {end_pos.z:.1f})")
                    print(f"  Jacobian condition number: {np.linalg.cond(J):.2f}")
                    
                    # Print joint positions for educational purposes
                    print(f"  Joint positions:")
                    for j in range(self.num_joints):
                        pos = self.joint_positions[j]
                        print(f"    Joint {j+1}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                
                # Update the GUI
                Gui.updateGui()
                

                # Save animation frame if requested
                if save_animation and Gui.ActiveDocument:
                    try:
                        frame_filename = os.path.join(output_folder, f"frame_{frame_count:04d}.png")
                        
                        # Get just the 3D view widget
                        view = Gui.ActiveDocument.ActiveView
                        
                        # METHOD 1: Using the view's saveImage method
                        try:
                            # This gets just the 3D view area
                            view.saveImage(frame_filename, 1920, 1080) # Adjust width/height as needed
                            print(f"  Saved frame {frame_count} using saveImage()")
                            frame_count += 1
                        except AttributeError:
                            # METHOD 2: Alternative approach - get the viewport directly
                            try:
                                viewport = view.getViewer().getViewport()
                                viewport.saveImage(frame_filename, 1920, 1080) # Adjust width/height as needed
                                print(f"  Saved frame {frame_count} using viewport.saveImage()")
                                frame_count += 1
                            except:
                                # METHOD 3: Try to identify and capture just the 3D view widget
                                try:
                                    # Find the 3D view widget within the main window
                                    main_window = Gui.getMainWindow()
                                    view_widget = None
                                    
                                    # Find the 3D view widget (it's usually a child of the main window)
                                    for widget in main_window.findChildren(QtGui.QWidget):
                                        if widget.objectName() == "View3DInventor":
                                            view_widget = widget
                                            break
                                    
                                    if view_widget:
                                        pixmap = QtGui.QPixmap.grabWidget(view_widget)
                                        pixmap.save(frame_filename)
                                        print(f"  Saved frame {frame_count} using grabWidget")
                                        frame_count += 1
                                    else:
                                        print("  Could not find 3D view widget")
                                except Exception as e:
                                    print(f"  METHOD 3 failed: {e}")
                                    
                                    # Fallback to full window as last resort
                                    try:
                                        pixmap = QtGui.QPixmap.grabWindow(Gui.getMainWindow().winId())
                                        pixmap.save(frame_filename)
                                        print(f"  Saved frame {frame_count} using window capture (fallback)")
                                        frame_count += 1
                                    except Exception as e2:
                                        print(f"  Fallback also failed: {e2}")
                    except Exception as e:
                        print(f"Error in frame saving process: {e}")



                    # Delay for animation
                    time.sleep(duration / (steps * 1.25))  # Slightly faster than specified


        if save_animation:
            print(f"\nAnimation complete! {frame_count} frames were saved to {output_folder}")
            print("\nTo convert frames to video on Ubuntu:")
            print("1. Using ffmpeg (install if needed with: sudo apt install ffmpeg):")
            print(f"   ffmpeg -framerate 30 -i {output_folder}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p robot_animation.mp4")
            print("\n2. Using avconv:")
            print(f"   avconv -framerate 30 -i {output_folder}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p robot_animation.mp4")
            
        print("Demonstration completed.")
        
        # Visualize end-effector path with a trail
        path_points = []
        
        # For each pose transition
        for i in range(len(poses) - 1):
            start_pose = np.array(poses[i])
            end_pose = np.array(poses[i+1])
            
            print(f"Moving from pose {i+1} to pose {i+2}...")
            
            # Interpolate between poses
            steps_for_segment = max(10, int(steps / (len(poses) - 1)))
            
            for step in range(steps_for_segment + 1):
                t = step / steps_for_segment
                
                # Smooth interpolation using sine curve for acceleration/deceleration
                smooth_t = 0.5 - 0.5 * np.cos(t * np.pi)
                
                # Linear interpolation with smooth timing
                current_angles = start_pose + smooth_t * (end_pose - start_pose)
                
                # Set joint angles
                self.joint_angles = current_angles
                
                # Update kinematics
                self.update_kinematics()
                
                # Update visualization
                self.update_freecad_objects()
                
                # Add end-effector position to path
                if step % 5 == 0:  # Only add every 5th point to keep the path cleaner
                    path_points.append(self.joint_positions[-1])
                
                # Print info occasionally
                if step % 10 == 0 or step == steps_for_segment:
                    # Calculate Jacobian to demonstrate use
                    J = self.calculate_jacobian()
                    
                    # Display interesting info
                    end_pos = self.joint_positions[-1]
                    print(f"  Step {step+1}/{steps_for_segment+1}")
                    print(f"  End-effector position: ({end_pos.x:.1f}, {end_pos.y:.1f}, {end_pos.z:.1f})")
                    print(f"  Jacobian condition number: {np.linalg.cond(J):.2f}")
                    
                    # FIXED: Print more details about arm configuration for debugging
                    print(f"  Joint positions:")
                    for j in range(self.num_joints):
                        pos = self.joint_positions[j]
                        print(f"    Joint {j+1}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                
                # Update the GUI
                Gui.updateGui()
                
                # Delay for animation
                time.sleep(duration / (steps * 1.25))  # Slightly faster than specified
        
        # Create a compound path visualization
        if path_points:
            path_shape = self.create_path_visualization(path_points)
            path_obj = App.ActiveDocument.addObject("Part::Feature", "EndEffectorPath")
            path_obj.Shape = path_shape
            if Gui.ActiveDocument:
                Gui.ActiveDocument.getObject("EndEffectorPath").LineColor = (1.0, 0.0, 0.0)  # Red
                Gui.ActiveDocument.getObject("EndEffectorPath").PointColor = (1.0, 1.0, 0.0)  # Yellow
                Gui.ActiveDocument.getObject("EndEffectorPath").LineWidth = 2
                Gui.ActiveDocument.getObject("EndEffectorPath").PointSize = 4
        
        print("Demonstration completed.")
    
    def create_path_visualization(self, points):
        """Create a visual path from a list of points"""
        if len(points) < 2:
            return None
            
        # Create polyline for path
        polyline = Part.makePolygon(points)
        
        # Create points for vertices
        point_shapes = [Part.Vertex(p) for p in points]
        
        # Create compound from all shapes
        compound = Part.makeCompound([polyline] + point_shapes)
        return compound

def main():
    """Main function to create and demonstrate robot arm functionality
    
    This function:
    1. Creates a robotic arm instance
    2. Initializes the arm in FreeCAD
    3. Displays key dimensions and initial configuration
    4. Runs a demonstration of direct joint control
    
    The demo shows how quaternions handle 3D rotations and how
    forward kinematics propagates these rotations through the kinematic chain.
    """
    print("==== Robotic Arm Simulation with Quaternions and Jacobian ====")
    print("Creating robot arm model in FreeCAD...")
    
    # Create the robotic arm
    robot = RoboticArm()
    robot.create_robot()
    
    # Wait a moment to let FreeCAD settle
    time.sleep(1)
    
    # Print educational information
    print("\n=== EDUCATIONAL NOTES ===")
    print("This simulation demonstrates three key robotics concepts:")
    print("1. QUATERNIONS: Used for smooth, gimbal-lock-free 3D rotations")
    print("2. FORWARD KINEMATICS: Computing positions from joint angles")
    print("3. JACOBIAN MATRIX: Relating joint velocities to end-effector velocities")
    print("======================\n")
    
    # Print debug info about robot dimensions
    print("Robot dimensions:")
    print(f"Base dimensions: {robot.base_length} x {robot.base_width} x {robot.base_height}")
    print(f"Joint dimensions: radius={robot.joint_radius}, height={robot.joint_height}")
    print(f"Link dimensions: width={robot.link_width}, height={robot.link_height}")
    print(f"Link lengths: {robot.link_lengths}")
    
    print("\nInitial kinematic chain configuration:")
    robot.update_kinematics()
    
    # Print joint positions before animation
    for i in range(robot.num_joints + 1):
        if i < robot.num_joints:
            print(f"Joint {i+1} position: ({robot.joint_positions[i].x:.1f}, {robot.joint_positions[i].y:.1f}, {robot.joint_positions[i].z:.1f})")
        else:
            print(f"End effector position: ({robot.joint_positions[i].x:.1f}, {robot.joint_positions[i].y:.1f}, {robot.joint_positions[i].z:.1f})")
    
    # Calculate and display the initial Jacobian
    J = robot.calculate_jacobian()
    print("\nInitial Jacobian matrix:")
    for row in J:
        print("  " + " ".join([f"{val:7.2f}" for val in row]))
    print(f"Condition number: {np.linalg.cond(J):.2f} (lower is better)")
    
    # Get user input for animation saving
    save_animation = False
    output_folder = None
    
    try:
        # Only ask if GUI is available
        if Gui.ActiveDocument:
            # Try to use PySide for a dialog box if available
            try:
                from PySide import QtGui
                app = QtGui.QApplication.instance()
                if not app:
                    app = QtGui.QApplication([])
                
                # Create message box
                msgBox = QtGui.QMessageBox()
                msgBox.setText("Would you like to save the animation?")
                msgBox.setInformativeText("This will save individual frames that can be converted to a video.")
                msgBox.setStandardButtons(QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
                msgBox.setDefaultButton(QtGui.QMessageBox.Yes)
                
                # Show dialog and get result
                if msgBox.exec_() == QtGui.QMessageBox.Yes:
                    save_animation = True
                    
                    # Let the user choose the output folder
                    folder_dialog = QtGui.QFileDialog()
                    folder_dialog.setWindowTitle("Select folder to save animation frames")
                    folder_dialog.setFileMode(QtGui.QFileDialog.Directory)
                    folder_dialog.setOption(QtGui.QFileDialog.ShowDirsOnly, True)
                    
                    if folder_dialog.exec_():
                        output_folder = folder_dialog.selectedFiles()[0]
                        print(f"Animation will be saved to: {output_folder}")
                    else:
                        # User cancelled folder selection
                        save_animation = False
            except ImportError:
                # Fall back to text prompt if PySide not available
                save_option = input("\nDo you want to save the animation as image frames? (y/n): ").strip().lower()
                if save_option.startswith('y'):
                    save_animation = True
                    print("Animation will be saved as a sequence of frames.")
    except Exception as e:
        print(f"Error setting up animation saving: {e}")
        # If there's any error, don't save animation
        save_animation = False
    
    # Run the direct joint control demo using quaternions and showing Jacobian computation
    print("\nStarting robot arm motion demonstration...")
    robot.demo_direct_joint_control(save_animation=save_animation, output_folder=output_folder)
    
    print("\nDemonstration completed. You can now manipulate the robot arm in FreeCAD.")
    print("Key concepts demonstrated:")
    print("- Quaternion rotation composition (joint_quat.multiply)")
    print("- Forward kinematics chain (update_kinematics)")
    print("- Jacobian calculation (calculate_jacobian)")
    
    # Return the robot object in case we want to manipulate it further
    return robot

def convert_frames_to_video(frames_folder=None, output_video=None, fps=30):
    """
    Convert a folder of image frames to a video using OpenCV.
    
    Uses PySide for GUI file dialogs if available.
    
    Parameters:
    - frames_folder: Path to folder containing the frames (optional, will prompt if None)
    - output_video: Path to save the output video (optional, will prompt if None)
    - fps: Frames per second for the output video
    
    Returns:
    - True if successful, False otherwise
    
    Note: This function requires OpenCV to be installed.
    To install in FreeCAD: Use the FreeCAD addon manager or run:
    - pip install opencv-python (from FreeCAD's Python)
    """
    try:
        import cv2
        import os
        import glob
        
        # Use PySide for folder selection if available and folder not specified
        if frames_folder is None or not os.path.exists(frames_folder):
            try:
                from PySide import QtGui
                app = QtGui.QApplication.instance()
                if not app:
                    app = QtGui.QApplication([])
                    
                # Select input folder
                folder_dialog = QtGui.QFileDialog()
                folder_dialog.setWindowTitle("Select folder with animation frames")
                folder_dialog.setFileMode(QtGui.QFileDialog.Directory)
                folder_dialog.setOption(QtGui.QFileDialog.ShowDirsOnly, True)
                
                if folder_dialog.exec_():
                    frames_folder = folder_dialog.selectedFiles()[0]
                    print(f"Selected frames folder: {frames_folder}")
                else:
                    print("No folder selected. Aborting video creation.")
                    return False
            except ImportError:
                # Fallback if PySide not available
                frames_folder = input("Enter path to folder containing frames: ")
                if not os.path.exists(frames_folder):
                    print(f"Folder not found: {frames_folder}")
                    return False
        
        # Get all frame filenames and sort them
        frame_files = sorted(glob.glob(os.path.join(frames_folder, "frame_*.png")))
        if not frame_files:
            print(f"No frames found in {frames_folder}")
            return False
        
        # Read the first frame to get dimensions
        first_frame = cv2.imread(frame_files[0])
        if first_frame is None:
            print(f"Could not read frame: {frame_files[0]}")
            return False
            
        height, width, _ = first_frame.shape
        
        # Use PySide for output file selection if available and file not specified
        if output_video is None:
            try:
                from PySide import QtGui
                app = QtGui.QApplication.instance()
                if not app:
                    app = QtGui.QApplication([])
                    
                # Select output video file
                file_dialog = QtGui.QFileDialog()
                file_dialog.setWindowTitle("Save video as")
                file_dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
                file_dialog.setNameFilter("Video Files (*.mp4 *.avi)")
                file_dialog.setDefaultSuffix("mp4")
                
                if file_dialog.exec_():
                    output_video = file_dialog.selectedFiles()[0]
                    print(f"Selected output file: {output_video}")
                else:
                    print("No output file selected. Aborting video creation.")
                    return False
            except ImportError:
                # Fallback if PySide not available
                output_video = input("Enter path to save output video (e.g. output.mp4): ")
        
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or 'XVID'
        out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
        
        # Write each frame to the video
        frame_count = len(frame_files)
        print(f"Converting {frame_count} frames to video...")
        
        for i, frame_file in enumerate(frame_files):
            frame = cv2.imread(frame_file)
            if frame is not None:
                out.write(frame)
                # Print progress every 10% of frames
                if i % max(1, frame_count // 10) == 0:
                    print(f"Progress: {i}/{frame_count} frames ({i/frame_count*100:.1f}%)")
            else:
                print(f"Warning: Could not read frame {frame_file}")
        
        # Release the VideoWriter
        out.release()
        print(f"Video saved successfully: {output_video}")
        return True
        
    except ImportError:
        print("Error: OpenCV (cv2) is not installed.")
        print("\nTo install OpenCV in FreeCAD:")
        print("1. In FreeCAD, go to Tools -> Addon Manager")
        print("2. Open the Python console (View -> Panels -> Python Console)")
        print("3. Run: import pip; pip.main(['install', 'opencv-python'])")
        print("\nOr from your system terminal:")
        print("1. Find FreeCAD's Python executable path:")
        print("   - In FreeCAD, open Python console and type: import sys; print(sys.executable)")
        print("2. Use that path to install OpenCV:")
        print("   - [FreeCAD's Python path] -m pip install opencv-python")
        return False
    except Exception as e:
        print(f"Error creating video: {e}")
        return False

if __name__ == '__main__':
    print("\n=== ROBOT ARM SIMULATION WITH QUATERNIONS AND JACOBIAN ===")
    print("\nInstallation and Requirements:")
    print("1. This script requires FreeCAD (https://www.freecadweb.org/)")
    print("2. For animation saving/conversion, PySide is included with FreeCAD")
    
    print("\n=== UBUNTU-SPECIFIC INSTRUCTIONS ===")
    print("Since you're using Ubuntu with FreeCAD from snap, here's how to convert animation frames:")
    print("1. Install ffmpeg if not already installed:")
    print("   sudo apt install ffmpeg")
    print("\n2. Use ffmpeg to convert frames to video:")
    print("   ffmpeg -framerate 30 -i ~/RobotArmAnimation/frame_%04d.png -c:v libx264 -pix_fmt yuv420p ~/robot_animation.mp4")
    print("\n3. Or use avconv (if you prefer it over ffmpeg):")
    print("   sudo apt install libav-tools (if not installed)")
    print("   avconv -framerate 30 -i ~/RobotArmAnimation/frame_%04d.png -c:v libx264 -pix_fmt yuv420p ~/robot_animation.mp4")
    
    print("\nNote: For snap-installed FreeCAD, the standard method to install Python packages won't work.")
    print("Instead, you can simply use the system's ffmpeg to convert the frames after they're saved.")
    
    print("\nTo use this script:")
    print("1. Open FreeCAD")
    print("2. Go to View -> Panels -> Python Console")
    print("3. Click the 'Open' button in the console")
    print("4. Navigate to and select this script file")
    print("5. Click 'Execute' to run the simulation")
    print("\nStarting simulation...\n")
    
    # Create and run the robot simulation
    robot = main()
    
    # Prompt user about converting frames to video if frames were saved
    try:
        import os
        
        # Check for animation frames in common locations
        home_dir = os.path.expanduser("~")
        default_frames_folder = os.path.join(home_dir, "RobotArmAnimation")
        
        # Try to find frames in the default folder or any recently created folder
        found_frames = False
        frames_folder = None
        
        if os.path.exists(default_frames_folder) and len(os.listdir(default_frames_folder)) > 0:
            found_frames = True
            frames_folder = default_frames_folder
        
        if found_frames:
            print("\n======= ANIMATION FRAMES DETECTED =======")
            print(f"Frames are saved in: {frames_folder}")
            print("\nTo convert these frames to a video on Ubuntu:")
            print(f"ffmpeg -framerate 30 -i {frames_folder}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p ~/robot_animation.mp4")
            print("\nCopy and paste this command into a terminal window (not in FreeCAD)")
            print("==================================")
            
            # Try to use PySide for a more user-friendly prompt
            try:
                from PySide import QtGui
                app = QtGui.QApplication.instance()
                if not app:
                    app = QtGui.QApplication([])
                
                msgBox = QtGui.QMessageBox()
                msgBox.setText("Animation frames were detected.")
                msgBox.setInformativeText(f"Frames are saved in: {frames_folder}\n\nTo convert to video, open a terminal and run:\nffmpeg -framerate 30 -i {frames_folder}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p ~/robot_animation.mp4")
                msgBox.setStandardButtons(QtGui.QMessageBox.Ok)
                msgBox.exec_()
            except ImportError:
                # Fallback dialog failed, we already printed the message above
                pass
    except Exception as e:
        print(f"An error occurred when processing video conversion: {e}")
    
    # Return the robot object in case it's needed
    # This makes the robot object available in the Python console
    # after running the script
    print("\nRobot object is available as 'robot' for further manipulation.")
    print("Example: robot.joint_angles = [30, 45, 20]  # Set joint angles")
    print("         robot.update_kinematics()          # Update positions")
    print("         robot.update_freecad_objects()     # Update visualization")
