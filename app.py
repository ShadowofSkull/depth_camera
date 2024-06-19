from flask import Flask, render_template, request
import rospy
from std_msgs.msg import String
from astra_camera.msg import MotorControl, GripperControl

app = Flask(__name__)

# Initialize ROS node
rospy.init_node('web_controller', anonymous=True)

# Publishers
pub_motor = rospy.Publisher('motor_control', MotorControl, queue_size=10)
pub_gripper = rospy.Publisher('gripper_control', GripperControl, queue_size=10)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    action = request.form['action']
    
    if action in ['o', 'c', 'flip', 'back']:
        gripper_control_msg = GripperControl()
        gripper_control_msg.state = action
        pub_gripper.publish(gripper_control_msg)
    else:
        motor_control_msg = MotorControl()
        if action == 'forward':
            motor_control_msg.motor1_speed = 100
            motor_control_msg.motor2_speed = 100
            motor_control_msg.motor3_speed = 100
            motor_control_msg.motor4_speed = 100
        elif action == 'backward':
            motor_control_msg.motor1_speed = -100
            motor_control_msg.motor2_speed = -100
            motor_control_msg.motor3_speed = -100
            motor_control_msg.motor4_speed = -100
        elif action == 'left':
            motor_control_msg.motor1_speed = 100
            motor_control_msg.motor2_speed = -100
            motor_control_msg.motor3_speed = 100
            motor_control_msg.motor4_speed = -100
        elif action == 'right':
            motor_control_msg.motor1_speed = -100
            motor_control_msg.motor2_speed = 100
            motor_control_msg.motor3_speed = -100
            motor_control_msg.motor4_speed = 100
        pub_motor.publish(motor_control_msg)

    return '', 204

if __name__ == '__main__':
    app.run(debug=True)

