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

**MUST: You have to change the hostname id in (combined_control_gui.py), and build the pkg again**

**change hostname id , ssid, password in (web-socket-arm.ino) file**

# Gamepad Controls

## Arm Control:
- **LT / RT**: Shared motor PWM slider  
- **LB / RB**: Wrist servo control  
- **D-pad Up**: Gripper open  
- **D-pad Down**: Gripper close  
- **D-pad Right**: Roller open  
- **D-pad Left**: Roller close  
- **L button**: Gripper stop  
- **R button**: Roller stop  
- **B**: Toggle base motor  
- **X**: Toggle shoulder motor  
- **Y**: Toggle elbow motor  
- **START**: Reset arm

## Wheel Control:
- **Left stick**: Direction control  
- **Right stick**: Speed adjustment  
- **START**: Stop all movement
