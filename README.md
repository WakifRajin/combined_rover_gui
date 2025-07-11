# Combined Rover Control System

## Prerequisites

- **ROS2** (Humble Hawksbill or newer recommended)
- **Python 3.8+**
- **System Dependencies**:
  ```bash
  sudo apt install python3-pip python3-pyqt5
  ```

## Installation

1. Create a ROS2 workspace
```bash
  mkdir -p ~/rover_ws/src
  cd ~/rover_ws/src
```
2. Clone this repository:
```bash
  git clone https://github.com/WakifRajin/combined_rover_gui.git
```
3. Install Python dependencies:
```bash
  cd ~/rover_ws
  pip3 install -r src/rover_gui/requirements.txt
```

4. Install ROS dependencies:
```bash
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

5. Build the package:
```bash
  colcon build --packages-select rover_gui
```

## Running the GUI

1. Source the workspace
```bash
  cd ~/rover_ws/
  source install/setup.bash
```

2. Run server.py manually (will be updated later) to start the websocket server

3. Start the GUI
```bash
  ros2 run rover_gui rover_gui
```
4. Run the .ino file (for arm)
**MUST: change hostname id in (gui.py) change hostname id , ssid, password in (.ino) file**

# Gamepad Controls

## Arm Control:
- **RT / LT**: Increase / Decrease PWM  
- **RB / LB**: Adjust wrist servo  
- **B / Y / X**: Toggle base / shoulder / elbow motors  
- **D-pad**: Control gripper and roller  
- **L / R buttons**: Stop gripper / roller  
- **START**: Reset arm

## Wheel Control:
- **Left stick**: Direction control  
- **Right stick**: Speed adjustment  
- **START**: Stop all movement


Right stick: Speed adjustment

START: Stop all movement
