# Robot Camera Web Server

A Flask-based web server that streams the robot's camera feed to a web browser, with real-time ball detection visualization.

## Features

- 🌐 **Web-based camera streaming** - View camera feed in any web browser
- 🎯 **Real-time ball detection** - Orange ball detection with visual indicators
- 📱 **Responsive interface** - Works on desktop and mobile devices
- 🔄 **Auto-refresh** - Keeps connection alive automatically
- 📸 **Snapshot capability** - Take screenshots of the camera feed
- 🔍 **Fullscreen mode** - View camera feed in fullscreen

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Start the Server

**Option A: Using the startup script (recommended)**
```bash
python start_camera_server.py
```

**Option B: Direct execution**
```bash
python camera_web_server.py
```

### 3. View the Camera Feed

1. Open your web browser
2. Navigate to: `http://localhost:5000`
3. The camera feed will appear with ball detection overlays

## Accessing from Other Devices

To view the camera feed from other devices on your network:

1. Find your Raspberry Pi's IP address:
   ```bash
   hostname -I
   ```

2. From another device, navigate to: `http://[YOUR_PI_IP]:5000`

## Camera Feed Features

### Visual Indicators
- **Green circle**: Detected orange ball with center point
- **White crosshair**: Frame center reference
- **Status text**: Shows ball coordinates or "No ball detected"

### Controls
- **🔄 Refresh Feed**: Reload the camera stream
- **🔍 Fullscreen**: View in fullscreen mode
- **📸 Take Snapshot**: Download a screenshot

### Ball Detection
- **Color Range**: Orange objects (HSV: 0-14, 132-255, 61-255)
- **Size Filter**: Objects between 100-30,000 pixels
- **Real-time**: Updates at ~30 FPS

## Technical Details

### Camera Configuration
- **Resolution**: 640x480
- **Format**: RGB888
- **Frame Rate**: ~30 FPS
- **Camera**: Pi Camera 2

### Server Configuration
- **Host**: 0.0.0.0 (accessible from network)
- **Port**: 5000
- **Threading**: Enabled for better performance

## Troubleshooting

### Camera Not Working
- Ensure the camera is connected and not being used by another application
- Check camera permissions and hardware connections
- Verify you're running on a Raspberry Pi with Pi Camera

### Web Server Issues
- Make sure port 5000 is not being used by another application
- Check firewall settings if accessing from other devices
- Verify Flask and other dependencies are installed correctly

### Ball Detection Issues
- Adjust the HSV color range in the code if needed
- Check lighting conditions
- Verify the orange object is within the size range (100-30,000 pixels)

## Files

- `camera_web_server.py` - Main web server implementation
- `start_camera_server.py` - Easy startup script
- `templates/camera_viewer.html` - Web interface
- `requirements.txt` - Python dependencies
- `README_camera_server.md` - This documentation

## Stopping the Server

Press `Ctrl+C` in the terminal to stop the web server gracefully.

## Integration with Robot Code

This web server can run alongside your robot code. The camera is shared between applications, so you can:

1. Run the web server for monitoring
2. Run your robot control code simultaneously
3. Use the web interface to debug ball detection
4. Monitor robot behavior in real-time

The web server uses the same camera initialization and ball detection logic as your main robot code for consistency.

