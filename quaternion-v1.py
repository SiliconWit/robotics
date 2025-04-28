import FreeCAD as App
import FreeCADGui as Gui
import Part
import math
import time

# Create a new document
doc = App.newDocument("QuaternionDemo")

# Create a transparent ground plane for reference
ground = Part.makePlane(100, 100, App.Vector(-50, -50, 0))
ground_obj = doc.addObject("Part::Feature", "Ground")
ground_obj.Shape = ground
ground_obj.ViewObject.ShapeColor = (0.8, 0.8, 0.8)
ground_obj.ViewObject.Transparency = 80  # Make it highly transparent

# Create a cube that we'll rotate using quaternions
cube = Part.makeBox(10, 10, 10, App.Vector(-5, -5, 5))
cube_obj = doc.addObject("Part::Feature", "RotatingCube")
cube_obj.Shape = cube
cube_obj.ViewObject.ShapeColor = (0.2, 0.6, 0.9)

# Create arrows to show the cube's local coordinate system
length = 15
x_axis = Part.makeCylinder(0.5, length, App.Vector(0, 0, 10), App.Vector(1, 0, 0))
x_obj = doc.addObject("Part::Feature", "X_Axis")
x_obj.Shape = x_axis
x_obj.ViewObject.ShapeColor = (1.0, 0.0, 0.0)  # Red

y_axis = Part.makeCylinder(0.5, length, App.Vector(0, 0, 10), App.Vector(0, 1, 0))
y_obj = doc.addObject("Part::Feature", "Y_Axis")
y_obj.Shape = y_axis
y_obj.ViewObject.ShapeColor = (0.0, 1.0, 0.0)  # Green

z_axis = Part.makeCylinder(0.5, length, App.Vector(0, 0, 10), App.Vector(0, 0, 1))
z_obj = doc.addObject("Part::Feature", "Z_Axis")
z_obj.Shape = z_axis
z_obj.ViewObject.ShapeColor = (0.0, 0.0, 1.0)  # Blue

# Set isometric view with improved zoom
Gui.ActiveDocument.ActiveView.viewIsometric()
Gui.SendMsgToActiveView("ViewFit")

# Adjust zoom to focus better on the cube
try:
    cam = Gui.ActiveDocument.ActiveView.getCameraNode()
    # Get current position
    pos = cam.position.getValue()
    # Move camera closer by scaling the position vector
    zoom_factor = 0.7  # Adjust this to zoom in (< 1) or out (> 1)
    new_pos = (pos[0] * zoom_factor, pos[1] * zoom_factor, pos[2] * zoom_factor)
    # Set new camera position
    cam.position.setValue(new_pos)
except:
    # Alternative zoom method if direct camera access fails
    for i in range(3):  # Zoom in 3 steps
        Gui.SendMsgToActiveView("ViewZoomIn")

# Recompute document
doc.recompute()

def quaternion_from_axis_angle(axis, angle_deg):
    """Create a quaternion from rotation axis and angle in degrees"""
    angle_rad = math.radians(angle_deg)
    
    # Normalize axis
    length = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    if length < 0.0001:
        return (1.0, 0.0, 0.0, 0.0)  # Identity quaternion
        
    axis = [axis[0]/length, axis[1]/length, axis[2]/length]
    
    # Calculate quaternion components
    s = math.sin(angle_rad/2)
    c = math.cos(angle_rad/2)
    
    # Return as (w, x, y, z) - note FreeCAD uses (x,y,z,w) order for its Rotation
    return (c, axis[0]*s, axis[1]*s, axis[2]*s)

def multiply_quaternions(q1, q2):
    """Multiply two quaternions (compose rotations)
    
    Both quaternions should be in (w, x, y, z) format.
    Returns the resulting quaternion in (w, x, y, z) format.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return (w, x, y, z)

def update_objects_rotation(rotation):
    """Update all objects with the given rotation"""
    # FreeCAD's rotation format is (x,y,z,w)
    x, y, z, w = rotation
    rot = App.Rotation(x, y, z, w)
    
    # Update cube
    cube_obj.Placement.Rotation = rot
    
    # Update coordinate axes
    x_obj.Placement.Rotation = rot
    y_obj.Placement.Rotation = rot
    z_obj.Placement.Rotation = rot
    
    # Recompute document
    doc.recompute()
    Gui.updateGui()

def demonstrate_quaternions():
    """Run a simple quaternion demonstration"""
    print("\n=== QUATERNION DEMONSTRATION ===")
    print("Quaternions provide a compact and efficient way to represent 3D rotations.")
    print("This demonstration will show basic quaternion operations.")
    
    # Set initial identity quaternion (no rotation)
    q_identity = (1.0, 0.0, 0.0, 0.0)  # w=1, x=y=z=0
    print("\nStarting with identity quaternion (no rotation):")
    print(f"q = ({q_identity[0]}, {q_identity[1]}, {q_identity[2]}, {q_identity[3]})")
    
    # To use with FreeCAD, we need (x,y,z,w) format
    update_objects_rotation((q_identity[1], q_identity[2], q_identity[3], q_identity[0]))
    time.sleep(1)
    
    # Demonstrate rotation around axes
    axes = [
        ([1, 0, 0], "X-axis"),  # X-axis
        ([0, 1, 0], "Y-axis"),  # Y-axis
        ([0, 0, 1], "Z-axis")   # Z-axis
    ]
    
    # Rotate around each axis 360 degrees
    for axis, name in axes:
        print(f"\nRotating 360° around the {name}:")
        
        for angle in range(0, 361, 45):  # in 45° increments
            # Create quaternion for this rotation
            q = quaternion_from_axis_angle(axis, angle)
            
            print(f"  Angle: {angle}°")
            print(f"  Quaternion: w={q[0]:.3f}, x={q[1]:.3f}, y={q[2]:.3f}, z={q[3]:.3f}")
            
            # Update the objects
            update_objects_rotation((q[1], q[2], q[3], q[0]))
            time.sleep(0.5)
    
    # Demonstrate quaternion combination (multiple rotations)
    print("\nDemonstrating quaternion multiplication (combining rotations):")
    
    # First rotation: 90° around X
    q1 = quaternion_from_axis_angle([1, 0, 0], 90)
    print("\nFirst rotation: 90° around X-axis")
    print(f"q1 = (w={q1[0]:.3f}, x={q1[1]:.3f}, y={q1[2]:.3f}, z={q1[3]:.3f})")
    
    # Apply first rotation
    update_objects_rotation((q1[1], q1[2], q1[3], q1[0]))
    time.sleep(1)
    
    # Second rotation: 90° around Y
    q2 = quaternion_from_axis_angle([0, 1, 0], 90)
    print("\nSecond rotation: 90° around Y-axis")
    print(f"q2 = (w={q2[0]:.3f}, x={q2[1]:.3f}, y={q2[2]:.3f}, z={q2[3]:.3f})")
    
    # Combine rotations
    q_combined = multiply_quaternions(q2, q1)  # Apply q1 then q2
    print("\nCombined rotation (q2 * q1, meaning q1 then q2):")
    print(f"q_combined = (w={q_combined[0]:.3f}, x={q_combined[1]:.3f}, y={q_combined[2]:.3f}, z={q_combined[3]:.3f})")
    
    # Apply combined rotation
    update_objects_rotation((q_combined[1], q_combined[2], q_combined[3], q_combined[0]))
    time.sleep(1)
    
    # Let's add a demonstration below the plane to show transparency
    print("\nDemonstrating rotation that moves cube below the plane:")
    
    # Rotation that puts the cube below the plane
    q_below = quaternion_from_axis_angle([1, 0, 0], 180)
    print(f"Rotation 180° around X-axis: (w={q_below[0]:.3f}, x={q_below[1]:.3f}, y={q_below[2]:.3f}, z={q_below[3]:.3f})")
    
    # Apply rotation
    update_objects_rotation((q_below[1], q_below[2], q_below[3], q_below[0]))
    time.sleep(2)
    
    # Demonstrate smooth rotation using interpolation
    print("\nDemonstrating smooth rotation between orientations:")
    
    # Start orientation (below plane)
    start_q = q_below
    
    # End orientation (some arbitrary orientation)
    end_q = quaternion_from_axis_angle([1, 1, 1], 120)  # Rotate around diagonal axis
    
    print(f"Starting quaternion: (w={start_q[0]:.3f}, x={start_q[1]:.3f}, y={start_q[2]:.3f}, z={start_q[3]:.3f})")
    print(f"Target quaternion: (w={end_q[0]:.3f}, x={end_q[1]:.3f}, y={end_q[2]:.3f}, z={end_q[3]:.3f})")
    
    # Interpolate between quaternions
    steps = 30
    for step in range(steps + 1):
        t = step / steps  # Interpolation factor [0..1]
        
        # Simple linear interpolation (not true SLERP, but illustrates the concept)
        w = start_q[0] * (1-t) + end_q[0] * t
        x = start_q[1] * (1-t) + end_q[1] * t
        y = start_q[2] * (1-t) + end_q[2] * t
        z = start_q[3] * (1-t) + end_q[3] * t
        
        # Normalize to ensure we have a unit quaternion
        length = math.sqrt(w*w + x*x + y*y + z*z)
        w /= length
        x /= length
        y /= length
        z /= length
        
        # Update objects
        update_objects_rotation((x, y, z, w))
        time.sleep(0.1)
    
    print("\nDemonstration complete!")
    print("\nKey advantages of quaternions:")
    print("1. No gimbal lock (unlike Euler angles)")
    print("2. Efficient representation (only 4 values)")
    print("3. Simple combination of rotations (quaternion multiplication)")
    print("4. Smooth interpolation between orientations")

# Run the demonstration
demonstrate_quaternions()