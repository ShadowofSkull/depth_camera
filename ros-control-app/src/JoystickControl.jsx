import React, { useState, useEffect } from 'react';
import Joystick from 'react-joystick';
import ROSLIB from 'roslib';

const JoystickControl = () => {
  const [ros, setRos] = useState(null); // ROS connection
  const [cmdVel, setCmdVel] = useState(null); // ROS topic publisher for cmd_vel

  // Connect to ROS
  useEffect(() => {
    const ros = new ROSLIB.Ros();
    ros.connect('ws://localhost:9090'); // Replace with your ROSBridge WebSocket URL

    ros.on('connection', () => {
      console.log('Connected to ROS');
      setRos(ros);

      // Create a publisher for cmd_vel topic
      const cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });
      setCmdVel(cmdVel);
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    return () => {
      ros.close(); // Disconnect from ROS on component unmount
      console.log('Disconnected from ROS');
      setRos(null);
      setCmdVel(null);
    };
  }, []);

  // Function to send Twist message
  const sendTwistCommand = (linearX, angularZ) => {
    if (cmdVel) {
      const twist = new ROSLIB.Message({
        linear: { x: linearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularZ },
      });
      cmdVel.publish(twist);
    }
  };

  // Handle joystick movement
  const handleMove = (event, data) => {
    const { angle, distance } = data;
    const speed = distance / 100; // Adjust speed factor as needed
    const angularZ = angle * Math.PI / 180; // Convert angle to radians for angular velocity

    // Calculate linearX based on joystick direction (forward/backward)
    let linearX = 0;
    if (angle > 45 && angle < 135) {
      linearX = speed;
    } else if (angle > 225 && angle < 315) {
      linearX = -speed;
    }

    // Send Twist message to ROS
    sendTwistCommand(linearX, angularZ);
  };

  return (
    <div>
      <h2>Joystick Control</h2>
      {!ros ? (
        <p>Connecting to ROS...</p>
      ) : (
        <div style={{ display: 'flex', justifyContent: 'center', marginTop: '20px' }}>
          <Joystick
            size={100}
            baseColor="#dddddd"
            stickColor="#555555"
            move={handleMove}
            stopPropagation={true}
          />
        </div>
      )}
    </div>
  );
};

export default JoystickControl;
