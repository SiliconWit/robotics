import FreeCAD as App
import FreeCADGui as Gui
import Part
import numpy as np
import time

# Create a new document
doc = App.newDocument("JacobianDemo")

# Create a simple 2-joint planar robot arm
def create_arm():
    # Base
    base = Part.makeBox(20, 20, 5, App.Vector(-10, -10, 0))
    base_obj = doc.addObject("Part::Feature", "Base")
    base_obj.Shape = base
    base_obj.ViewObject.ShapeColor = (0.2, 0.2, 0.2)
    
    # Joint 1
    joint1 = Part.makeSphere(5)
    joint1_obj = doc.addObject("Part::Feature", "Joint1")
    joint1_obj.Shape = joint1
    joint1_obj.Placement.Base = App.Vector(0, 0, 5)
    joint1_obj.ViewObject.ShapeColor = (0.7, 0.1, 0.1)
    
    # Link 1
    link1 = Part.makeCylinder(3, 50, App.Vector(0, 0, 0), App.Vector(1, 0, 0))
    link1_obj = doc.addObject("Part::Feature", "Link1")
    link1_obj.Shape = link1
    link1_obj.Placement.Base = App.Vector(0, 0, 5)
    link1_obj.ViewObject.ShapeColor = (0.1, 0.5, 0.8)
    
    # Joint 2
    joint2 = Part.makeSphere(4)
    joint2_obj = doc.addObject("Part::Feature", "Joint2")
    joint2_obj.Shape = joint2
    joint2_obj.Placement.Base = App.Vector(50, 0, 5)
    joint2_obj.ViewObject.ShapeColor = (0.7, 0.1, 0.1)
    
    # Link 2
    link2 = Part.makeCylinder(2, 40, App.Vector(0, 0, 0), App.Vector(1, 0, 0))
    link2_obj = doc.addObject("Part::Feature", "Link2")
    link2_obj.Shape = link2
    link2_obj.Placement.Base = App.Vector(50, 0, 5)
    link2_obj.ViewObject.ShapeColor = (0.1, 0.7, 0.7)
    
    # End effector
    end_effector = Part.makeSphere(3)
    ee_obj = doc.addObject("Part::Feature", "EndEffector")
    ee_obj.Shape = end_effector
    ee_obj.Placement.Base = App.Vector(90, 0, 5)
    ee_obj.ViewObject.ShapeColor = (0.1, 0.8, 0.1)
    
    # Set isometric view and fit to view
    Gui.ActiveDocument.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
    
    # Return objects for manipulation
    return joint1_obj, link1_obj, joint2_obj, link2_obj, ee_obj

# Create robot arm
joint1, link1, joint2, link2, end_effector = create_arm()
doc.recompute()

# Link lengths
L1 = 50
L2 = 40

def update_robot_position(theta1, theta2):
    """Update robot position for given joint angles (in degrees)
    
    Parameters:
    - theta1: Angle of the first joint (degrees)
    - theta2: Angle of the second joint relative to the first link (degrees)
    
    Returns:
    - (x_ee, y_ee): Position of the end effector
    """
    # Convert to radians
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    # Update joint 1 and link 1
    joint1_rot = App.Rotation(App.Vector(0, 0, 1), theta1)
    joint1.Placement.Rotation = joint1_rot
    link1.Placement.Rotation = joint1_rot
    
    # Calculate joint 2 position
    x2 = L1 * np.cos(theta1_rad)
    y2 = L1 * np.sin(theta1_rad)
    
    # Update joint 2 and link 2
    joint2.Placement.Base = App.Vector(x2, y2, 5)
    joint2.Placement.Rotation = joint1_rot
    
    link2.Placement.Base = App.Vector(x2, y2, 5)
    link2_rot = App.Rotation(App.Vector(0, 0, 1), theta1 + theta2)
    link2.Placement.Rotation = link2_rot
    
    # Calculate end effector position
    x_ee = x2 + L2 * np.cos(theta1_rad + theta2_rad)
    y_ee = y2 + L2 * np.sin(theta1_rad + theta2_rad)
    
    # Update end effector
    end_effector.Placement.Base = App.Vector(x_ee, y_ee, 5)
    
    # Recompute
    doc.recompute()
    
    return x_ee, y_ee

def calculate_jacobian(theta1, theta2):
    """Calculate the Jacobian matrix for 2R planar robot
    
    The Jacobian matrix maps joint velocities to end-effector velocities:
    [ ẋ ]   [ J11  J12 ] [ θ̇1 ]
    [ ẏ ] = [ J21  J22 ] [ θ̇2 ]
    
    For a 2-joint planar robot:
    J11 = -L1*sin(θ1) - L2*sin(θ1+θ2)
    J12 = -L2*sin(θ1+θ2)
    J21 = L1*cos(θ1) + L2*cos(θ1+θ2)
    J22 = L2*cos(θ1+θ2)
    """
    # Convert to radians
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    # Jacobian matrix elements
    J11 = -L1 * np.sin(theta1_rad) - L2 * np.sin(theta1_rad + theta2_rad)
    J12 = -L2 * np.sin(theta1_rad + theta2_rad)
    J21 = L1 * np.cos(theta1_rad) + L2 * np.cos(theta1_rad + theta2_rad)
    J22 = L2 * np.cos(theta1_rad + theta2_rad)
    
    # Return Jacobian matrix
    return np.array([[J11, J12], 
                     [J21, J22]])

def demo_jacobian():
    """Demonstrate the Jacobian matrix with the robot arm"""
    print("\n=== Jacobian Matrix Demonstration ===")
    print("This demonstration will show how the Jacobian matrix relates")
    print("joint velocities to end-effector velocities in a robot arm.\n")
    
    # Test positions
    test_positions = [
        (0, 0),    # Fully extended
        (45, 0),   # Base rotated 45°
        (0, 90),   # Elbow bent a 90°
        (45, 45),  # Both joints at 45°
        (90, 0),   # Base at 90°
        (135, -90) # Folded back
    ]
    
    # Ensure the view is properly set
    Gui.ActiveDocument.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
    
    for theta1, theta2 in test_positions:
        print(f"\nJoint angles: θ1 = {theta1}°, θ2 = {theta2}°")
        
        # Update robot position
        x_ee, y_ee = update_robot_position(theta1, theta2)
        print(f"End effector position: ({x_ee:.2f}, {y_ee:.2f})")
        
        # Calculate Jacobian
        J = calculate_jacobian(theta1, theta2)
        print("Jacobian matrix:")
        print(f"J = [[{J[0,0]:.2f}, {J[0,1]:.2f}],")
        print(f"     [{J[1,0]:.2f}, {J[1,1]:.2f}]]")
        
        # Explanation of Jacobian elements
        print("\nWhat the Jacobian elements mean:")
        print(f"J11 = {J[0,0]:.2f}: Effect of joint 1 velocity on x-direction motion")
        print(f"J12 = {J[0,1]:.2f}: Effect of joint 2 velocity on x-direction motion")
        print(f"J21 = {J[1,0]:.2f}: Effect of joint 1 velocity on y-direction motion")
        print(f"J22 = {J[1,1]:.2f}: Effect of joint 2 velocity on y-direction motion")
        
        # Calculate determinant (measure of manipulability)
        det_J = np.linalg.det(J)
        print(f"\nDeterminant of J: {det_J:.2f}")
        
        if abs(det_J) < 1.0:
            print("WARNING: Near singularity - robot loses DOF")
            print("In this configuration, certain end-effector movements")
            print("would require extremely high joint velocities.")
        
        # Calculate condition number (another measure of manipulability)
        try:
            cond = np.linalg.cond(J)
            print(f"Condition number of J: {cond:.2f}")
            
            if cond > 10:
                print("Poor manipulation capability in some directions")
                print("The robot will struggle to move precisely in certain directions.")
            else:
                print("Good manipulation capability in all directions")
                print("The robot can move efficiently in any direction.")
        except:
            print("Could not compute condition number - likely at singularity")
        
        # Demonstrate velocity mapping
        joint_velocity = np.array([10, 10])  # degrees/sec for each joint
        ee_velocity = J @ np.radians(joint_velocity)  # Convert to radians
        print(f"\nVelocity mapping example:")
        print(f"Joint velocities: [{joint_velocity[0]}, {joint_velocity[1]}] deg/s")
        print(f"End effector velocity: [{ee_velocity[0]:.2f}, {ee_velocity[1]:.2f}] units/s")
        
        # Pause to let user see configuration
        Gui.updateGui()
        time.sleep(2)
    
    # Show path with small Jacobian determinant (near singularity)
    print("\nDemonstrating approach to singularity...")
    print("Watch how the determinant changes as the arm straightens:")
    
    # Start from bent position
    theta1 = 0
    theta2 = 90
    update_robot_position(theta1, theta2)
    
    # Slowly straighten the arm
    for angle in range(90, -1, -10):
        theta2 = angle
        x_ee, y_ee = update_robot_position(theta1, theta2)
        J = calculate_jacobian(theta1, theta2)
        det_J = np.linalg.det(J)
        
        print(f"θ1 = {theta1}°, θ2 = {theta2}°, |J| = {det_J:.2f}")
        
        Gui.updateGui()
        time.sleep(0.5)
    
    # Additional educational explanation
    print("\nKey insights about the Jacobian matrix:")
    print("1. The Jacobian maps joint velocities to end-effector velocities")
    print("2. Each column represents the effect of one joint's movement")
    print("3. Singularities occur when the determinant approaches zero")
    print("4. At singularities, the robot loses one or more degrees of freedom")
    print("5. The condition number indicates how evenly the robot can move in all directions")
    
    # Final position for display
    update_robot_position(45, 45)
    Gui.updateGui()
    
    print("\nDemonstration complete!")

# Run the demonstration
demo_jacobian()