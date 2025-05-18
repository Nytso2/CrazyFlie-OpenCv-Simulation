[![Watch the demo](https://i.imgur.com/9nGdC5j.png)](https://www.youtube.com/watch?v=9ssjhqC-uFs)


# Crazyflie AI and Ball Supervisor

This repository contains code for controlling a Crazyflie drone and managing a ball in Webots using Python. The system uses computer vision, PID control, and keyboard input to enable movement and tracking tasks.

## Files

- `CrazyFlieAI.py`: Controls the Crazyflie drone, using computer vision to detect and track a green ball. It utilizes a PID controller for smooth movement and maintains a fixed height while adjusting yaw and forward motion based on the ball's position.
  
- `pid_controller.py`: Implements a PID controller for controlling the Crazyflie drone's velocity and altitude, ensuring smooth and precise motion.
  
- `ball_supervisor.py`: A Webots Supervisor controller script that allows manual movement of a ball in the simulation using the keyboard. The ball can be moved in different directions and rotated.

- `crazyflie_apartment.wbt`: A Webots world file that includes an environment setup for the Crazyflie drone to operate in.

## Features

- **Ball Control**: Move the ball in Webots using arrow keys or W/S for height control and Q/E for yaw rotation.
- **Green Ball Tracking**: The drone automatically moves toward a green ball detected by its camera, adjusting speed based on the ball's size.
- **PID Control**: The PID controller ensures stable flight for the Crazyflie drone, including maintaining altitude and responding to changes in position and orientation.

## Requirements

- Webots (for simulation)
- Python 3.x
- OpenCV (for computer vision)
- Numpy (for numerical operations)

You can install the required Python packages using:

```bash
pip install numpy
pip install opencv-python numpy
```

# Usage
- Open the crazyflie_apartment.wbt world in Webots.
- Set the controller for the drone to be CrazyFlieAI.py.
- Play around with the arrow keys or W/S for height control and Q/E for yaw rotation and look how the drone follows the ball!


