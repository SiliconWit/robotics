import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib import gridspec

# ======== EXPERIMENT: CHANGE THESE VALUES ========
# Try adjusting these values to see how they affect the robot arm!

# Robot arm dimensions
L1 = 1.0  # Length of first arm segment
L2 = 0.8  # Length of second arm segment

# Starting joint angles (in degrees)
angle1 = 45  # First joint angle (degrees)
angle2 = 60  # Second joint angle (degrees)

# ===============================================================

# Convert angles to radians for calculations
theta1 = np.radians(angle1)
theta2 = np.radians(angle2)

# Function to calculate Jacobian matrix for a 2-joint planar robot arm
def calculate_jacobian(theta1, theta2, L1, L2):
    """
    Calculate the Jacobian matrix for a 2-joint robot arm.
    
    The Jacobian relates joint velocities to end-effector velocities:
    [ẋ]   [J11 J12] [θ̇1]
    [ẏ] = [J21 J22] [θ̇2]
    
    When the determinant of J is zero, we have a singularity!
    """
    # Calculate end-effector position
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    
    # Jacobian matrix elements - these show how joint movements affect end-effector movement
    J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)  # How θ1 affects x
    J12 = -L2 * np.sin(theta1 + theta2)                        # How θ2 affects x
    J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)   # How θ1 affects y
    J22 = L2 * np.cos(theta1 + theta2)                         # How θ2 affects y
    
    # Form the Jacobian matrix
    J = np.array([[J11, J12], [J21, J22]])
    
    # Calculate determinant to check for singularity
    # If determinant is zero, the matrix is not invertible -> singularity!
    det = np.linalg.det(J)
    
    return J, det, (x, y)

# Function to visualize robot arm and movement possibilities
def plot_robot_arm(ax, theta1, theta2, L1, L2):
    ax.clear()
    
    # Calculate joint positions
    x0 = 0
    y0 = 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    # Plot arm segments
    ax.plot([x0, x1], [y0, y1], 'b-', linewidth=4, label='Link 1')
    ax.plot([x1, x2], [y1, y2], 'g-', linewidth=4, label='Link 2')
    
    # Plot joints
    ax.plot(x0, y0, 'ko', markersize=10, label='Base')
    ax.plot(x1, y1, 'ro', markersize=8, label='Joint 1')
    ax.plot(x2, y2, 'mo', markersize=8, label='End-effector')
    
    # Calculate Jacobian and determinant
    J, det, _ = calculate_jacobian(theta1, theta2, L1, L2)
    
    # Generate movement directions - visualize where the end-effector can move
    scale = 0.2
    
    # Show a circle or line based on determinant (mobility)
    if abs(det) < 0.01:  # Near singularity
        # For singular case, create a line (reduced mobility)
        direction = np.array([J[0,0], J[1,0]])  # Direction from first column of Jacobian
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm
            t = np.linspace(-1, 1, 100)
            # Show line of possible movement
            ax.plot(x2 + direction[0]*scale*t, y2 + direction[1]*scale*t, 
                   'r-', linewidth=2, label=f'RESTRICTED MOVEMENT\nDeterminant ≈ {det:.3f}')
    else:  # Non-singular
        # Show full circle of movement possibilities
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = x2 + scale * np.cos(theta)
        circle_y = y2 + scale * np.sin(theta)
        ax.plot(circle_x, circle_y, 'g--', linewidth=2, 
               label=f'FULL MOVEMENT\nDeterminant = {det:.3f}')
    
    # Calculate the workspace boundary (all possible positions of the end-effector)
    theta1_range = np.linspace(0, 2*np.pi, 100)
    theta2_range = np.linspace(0, 2*np.pi, 20)
    
    # Plot workspace boundary
    for t2 in theta2_range:
        x_boundary = L1 * np.cos(theta1_range) + L2 * np.cos(theta1_range + t2)
        y_boundary = L1 * np.sin(theta1_range) + L2 * np.sin(theta1_range + t2)
        ax.plot(x_boundary, y_boundary, 'c:', alpha=0.2)
    
    # Set labels and title
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    # Show singularity status
    if abs(det) < 0.01:
        title = "SINGULARITY DETECTED!"
        ax.set_title(title, color='red', fontsize=16)
    else:
        title = "Normal Configuration"
        ax.set_title(title, color='green', fontsize=16)
    
    ax.grid(True)
    ax.legend(loc='upper right')
    
    # Set axis limits with some margin
    max_reach = L1 + L2
    ax.set_xlim(-max_reach*1.2, max_reach*1.2)
    ax.set_ylim(-max_reach*1.2, max_reach*1.2)
    
    # Add explanation text
    if abs(det) < 0.01:
        # Explain what's happening in a singularity
        ax.text(0, -1.5, "SINGULARITY: The robot arm cannot move in all directions!\n"
                        "The end-effector can only move along the red line.\n"
                        "This happens when the arm is fully extended or folded.", 
                ha='center', color='red', bbox=dict(facecolor='white', alpha=0.8))
    else:
        # Explain normal configuration
        ax.text(0, -1.5, "NORMAL: The robot arm can move in all directions.\n"
                        "The end-effector can move anywhere along the green circle.\n"
                        "The Jacobian matrix is invertible (non-zero determinant).",
                ha='center', color='green', bbox=dict(facecolor='white', alpha=0.8))
    
    # Show the Jacobian matrix
    matrix_text = f"Jacobian Matrix:\n"
    matrix_text += f"[{J[0,0]:.2f}  {J[0,1]:.2f}]\n"
    matrix_text += f"[{J[1,0]:.2f}  {J[1,1]:.2f}]\n\n"
    matrix_text += f"Determinant: {det:.4f}"
    
    # If we're near a singularity, explain why
    if abs(det) < 0.01:
        if abs(theta2) < 0.1 or abs(theta2 - np.pi) < 0.1:
            matrix_text += "\n\nWHY? The arm is stretched out or folded back."
            matrix_text += "\nThe second row of the Jacobian is nearly"
            matrix_text += "\nproportional to the first row."
    
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax.text(1.1, 0.5, matrix_text, transform=ax.transAxes, fontsize=10,
           verticalalignment='center', bbox=props)
    
    # Return updated values for display
    return det, (x2, y2)

# Create the main figure and subplots
fig = plt.figure(figsize=(10, 8))
gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
ax = plt.subplot(gs[0])
ax_slider = plt.subplot(gs[1])

# Initial plot
det, pos = plot_robot_arm(ax, theta1, theta2, L1, L2)

# Add sliders for interactive exploration
ax_slider.set_facecolor('lightgoldenrodyellow')

# Create slider axes
ax_theta1 = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_theta2 = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_reset = plt.axes([0.8, 0.02, 0.1, 0.04])

# Create sliders
s_theta1 = Slider(ax_theta1, 'Joint 1 (degrees)', 0, 360, valinit=angle1)
s_theta2 = Slider(ax_theta2, 'Joint 2 (degrees)', 0, 360, valinit=angle2)
button_reset = Button(ax_reset, 'Reset', color='lightgoldenrodyellow')

# Define update functions
def update(val):
    # Get current values from sliders
    theta1 = np.radians(s_theta1.val)
    theta2 = np.radians(s_theta2.val)
    
    # Update plot
    det, pos = plot_robot_arm(ax, theta1, theta2, L1, L2)
    
    # Redraw
    fig.canvas.draw_idle()

def reset(event):
    # Reset sliders to initial values
    s_theta1.reset()
    s_theta2.reset()

# Register callbacks
s_theta1.on_changed(update)
s_theta2.on_changed(update)
button_reset.on_clicked(reset)

# Add title to explain the purpose
plt.figtext(0.5, 0.97, 
           "Robot Arm Singularity Explorer\n"
           "Move the sliders to change joint angles and observe singularities",
           ha='center', fontsize=14, bbox=dict(facecolor='white', alpha=0.8))

# Add explanation of the Jacobian and singularities at the bottom
singularity_explanation = """
Jacobian Matrix: Relates joint movements to end-effector movements
Singularity: When determinant = 0, the arm loses mobility in certain directions
EXPERIMENT: Find positions where the determinant becomes zero!
"""
plt.figtext(0.5, 0.01, singularity_explanation, ha='center', va='bottom', fontsize=10, 
           bbox=dict(facecolor='lightblue', alpha=0.8))

# Show the plot
plt.tight_layout(rect=[0, 0.05, 1, 0.95])
plt.show()

# Simple test cases to understand determinants
print("\n===== UNDERSTANDING DETERMINANTS =====")
print("A matrix with non-zero determinant is invertible.")
print("A matrix with zero determinant is NOT invertible.")

print("\nExample 1: Regular matrix")
A = np.array([[3, 1], [2, 2]])
print(f"Matrix A = \n{A}")
print(f"Determinant of A = {np.linalg.det(A)}")
print("Can we invert A?", "Yes!" if np.linalg.det(A) != 0 else "No!")

print("\nExample 2: Singular matrix")
B = np.array([[3, 6], [1, 2]])
print(f"Matrix B = \n{B}")
print(f"Determinant of B = {np.linalg.det(B)}")
print("Can we invert B?", "Yes!" if np.linalg.det(B) != 0 else "No!")
print("Note: Columns of B are proportional! (Column 2 = 2 × Column 1)")

print("\n===== ROBOT ARM INSIGHTS =====")
print("When a robot arm has a singular Jacobian matrix:")
print("1. It cannot move in all directions")
print("2. The control system may become unstable")
print("3. Certain tasks become impossible")
print("\nCommon singularity positions:")
print("- When the arm is fully extended (θ2 ≈ 0°)")
print("- When the arm is folded back (θ2 ≈ 180°)")