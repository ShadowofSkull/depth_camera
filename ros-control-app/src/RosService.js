import ROSLIB from 'roslib';

class RosService {
  constructor() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Adjust port if necessary
    });

    // Example topic for movement commands (adjust as per your robot's setup)
    this.cmdVelTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
  }

  connect() {
    this.ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    this.ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    this.ros.on('close', () => {
      console.log('Disconnected from ROS');
    });

    this.ros.connect();
  }

  sendMovementCommand(linearX, angularZ) {
    const twist = new ROSLIB.Message({
      linear: {
        x: linearX,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angularZ
      }
    });

    this.cmdVelTopic.publish(twist);
  }
}

export default RosService;
