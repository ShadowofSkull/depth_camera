import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Create figure and axis
fig, ax = plt.subplots(figsize=(20, 18))

# Function to add a box with text
def add_box(text, xy, boxstyle='round,pad=0.5', boxcolor='lightblue'):
    box = patches.FancyBboxPatch(xy, 4, 1, boxstyle=boxstyle, linewidth=1, edgecolor='black', facecolor=boxcolor)
    ax.add_patch(box)
    ax.text(xy[0] + 2, xy[1] + 0.5, text, ha='center', va='center', fontsize=12)

# Function to add an arrow
def add_arrow(start, end):
    ax.annotate('', xy=end, xytext=start, arrowprops=dict(arrowstyle="->", lw=1.5))

# Add boxes
add_box('detect_silo.py detects ball\n(x, z coordinates)', (0, 10))
add_box('MotorControl.ino receives\n(x, z values) and moves', (0, 8))
add_box('GripperControl.ino detects\nball with IR sensor', (0, 6))
add_box('Purple Ball Detected:\nGrip, Flip, Release,\nand Flip Back', (-5, 4))
add_box('Blue Ball Detected:\nGrip, Flip, Detect Silo\nConditions, Move to Best Silo,\nand Release', (5, 4))
add_box('ultrasonic_sensor.py measures\nultrasonic values', (0, 2))
add_box('detect_silo.py reads ultrasonic\nvalues and positions robot', (0, 0))
add_box('Robot places ball into\ncorrect silo', (0, -2))
add_box('Gripper flips back to\ndetect the next ball', (0, -4))

# Add arrows
add_arrow((2, 10), (2, 9))
add_arrow((2, 8), (2, 7))
add_arrow((2, 6), (2, 5))
add_arrow((2, 4), (2, 3))
add_arrow((2, 2), (2, 1))
add_arrow((2, 0), (2, -1))
add_arrow((2, -2), (2, -3))

# Conditional arrows
add_arrow((2, 6), (-3, 5))
add_arrow((2, 6), (7, 5))

# Set axis limits and hide axes
ax.set_xlim(-20, 40)
ax.set_ylim(-30, 20)
ax.axis('on')

plt.show()
