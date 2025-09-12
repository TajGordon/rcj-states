# Funzies - Remote Control Interface

This directory contains experimental and fun features for the robot, including a remote control interface.

## Remote Control

The remote control interface provides:

### Features
- **Real-time camera feed** with overlay information
- **WASD movement controls** (W=forward, S=backward, A=left, D=right)
- **QE rotation controls** (Q=left turn, E=right turn)
- **Variable speed control** (1-8 levels)
- **Field visualization** showing robot position and ToF sensor rays
- **Real-time robot data** including position, heading, ball detection, etc.
- **Web-based interface** accessible from any device on the network

### Usage

1. **Start the remote control:**
   ```bash
   cd good/funzies
   python remote_control.py
   ```

2. **Open your web browser:**
   ```
   http://localhost:5001
   ```
   Or from another device on the network:
   ```
   http://[robot-ip]:5001
   ```

3. **Control the robot:**
   - Use WASD keys or click the control buttons
   - Adjust speed with the slider (1-8)
   - Use Q/E for rotation
   - Release keys to stop movement

### Controls

| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Move left (strafe) |
| D | Move right (strafe) |
| Q | Turn left |
| E | Turn right |
| Speed Slider | Adjust movement speed (1-8) |

### Data Display

The interface shows:
- **Robot position** (x, y coordinates in mm)
- **Robot heading** (degrees)
- **Ball detection status** and position
- **Ball distance** from camera center
- **ToF sensor readings** (angle and distance)
- **Localization error** (position estimation accuracy)
- **Field visualization** with robot and ball markers

### Technical Details

- **Web server**: Flask with SocketIO for real-time communication
- **Video streaming**: MJPEG stream from robot camera
- **Control frequency**: 20 Hz control loop
- **Data update**: 10 Hz robot data collection
- **Video frame rate**: ~30 FPS

### Requirements

- Flask
- Flask-SocketIO
- OpenCV (cv2)
- All robot components (camera, motors, IMU, ToF, localization)

### Safety Notes

- The robot will move immediately when controls are pressed
- Make sure there's adequate space around the robot
- Use lower speed levels when testing
- The robot will stop when keys are released
- Emergency stop: Close the browser or press Ctrl+C in terminal
