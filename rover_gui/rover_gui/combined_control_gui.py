import sys
import numpy as np
import threading
import websocket
import json
import logging
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QSlider, QGroupBox, QMainWindow, QButtonGroup,
    QSplitter, QFrame, QSizePolicy, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Arrow, Circle

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Arm segment lengths
L1, L2, L3 = 10, 10, 5

class DetachedPlotWindow(QMainWindow):
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        self.setWindowTitle("Detached Plot")
        self.setGeometry(300, 300, 600, 600)
        self.setCentralWidget(gui.canvas)

    def closeEvent(self, event):
        self.gui.reattach_plot()
        event.accept()

class ArmControlGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setup_ui()
        
        # Control variables
        self.gripper_state = 0  # 0: stop, 1: close, 2: open
        self.roller_state = 0   # 0: stop, 1: close, 2: open
        self.servo_angle = 90
        self.elbow_pwm = 0
        self.shoulder_pwm = 0
        self.base_pwm = 0
        self.shared_pwm = 0
        self.last_values = None

        # Motor states (0: stop, 1: forward, 2: backward)
        self.base_state = 0
        self.shoulder_state = 0
        self.elbow_state = 0

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignTop)

        # Connection status
        self.status_label = QLabel("🔴 Disconnected")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        layout.addWidget(self.status_label)
        
        # Gamepad status
        self.gamepad_status = QLabel("🔴 No gamepad")
        self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px;")
        layout.addWidget(self.gamepad_status)

        # Shared PWM slider for base, shoulder, and elbow motors
        self.shared_pwm_slider = self.create_pwm_slider("Shared Motor PWM", lambda val: setattr(self, 'shared_pwm', val), 0, 1023)
        layout.addWidget(self.shared_pwm_slider)
        
        # Motor control sections
        layout.addWidget(self.create_motor_control("Base Motor", 'base'))
        layout.addWidget(self.create_motor_control("Shoulder Motor", 'shoulder'))
        layout.addWidget(self.create_motor_control("Elbow Motor", 'elbow'))
        
        # Servo control
        self.servo_slider = self.create_pwm_slider("Wrist Servo (0-180°)", lambda val: setattr(self, 'servo_angle', val), 0, 180)
        layout.addWidget(self.servo_slider)
        
        # Gripper and Roller controls
        layout.addWidget(self.create_gripper_roller_control("Gripper", 'gripper'))
        layout.addWidget(self.create_gripper_roller_control("Roller", 'roller'))

        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("Reset")
        reset_btn.setMinimumHeight(40)
        reset_btn.clicked.connect(self.reset_all)

        self.detach_btn = QPushButton("Detach Plot")
        self.detach_btn.setMinimumHeight(40)
        self.detach_btn.clicked.connect(self.parent.toggle_arm_plot_detach)

        btn_layout.addWidget(reset_btn)
        btn_layout.addWidget(self.detach_btn)
        layout.addLayout(btn_layout)

        self.setStyleSheet("""
            QWidget { background-color: #f4f4f4; font-family: 'Segoe UI', Arial; font-size: 12pt; }
            QGroupBox { border: 1px solid #cccccc; border-radius: 8px; margin-top: 1.5ex; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background-color: #2b2d42; color: white; border-radius: 8px; padding: 8px; font-weight: bold; }
            QPushButton:hover { background-color: #1f2235; }
            QPushButton:checked { background-color: #4a4d6d; }
            QSlider::groove:horizontal { background: #cccccc; height: 10px; border-radius: 5px; }
            QSlider::handle:horizontal { background: #2b2d42; width: 30px; height: 30px; border-radius: 15px; margin: -10px 0; }
        """)

    def create_pwm_slider(self, label, callback, min_val=0, max_val=1023):
        group = QGroupBox(label)
        layout = QVBoxLayout()
        
        # Add value label
        value_label = QLabel(f"{(min_val + max_val) // 2}")
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setStyleSheet("font-weight: bold; color: #2b2d42;")
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue((min_val + max_val) // 2)
        slider.setMinimumHeight(35)
        
        def on_value_change(val):
            value_label.setText(str(val))
            callback(val)
        
        slider.valueChanged.connect(on_value_change)
        
        layout.addWidget(value_label)
        layout.addWidget(slider)
        group.setLayout(layout)
        
        # Store slider and label for gamepad updates
        if label == "Shared Motor PWM":
            self.shared_pwm_label = value_label
            self.shared_pwm_slider_ref = slider
        elif label == "Wrist Servo (0-180°)":
            self.servo_label = value_label
            self.servo_slider_ref = slider
            
        return group

    def create_motor_control(self, name, motor_type):
        group = QGroupBox(name)
        layout = QHBoxLayout()
        
        # Forward button
        fwd_btn = QPushButton("FWD")
        fwd_btn.setCheckable(True)
        fwd_btn.setMinimumHeight(40)
        fwd_btn.clicked.connect(lambda: self.set_motor_state(motor_type, 1))
        
        # Backward button
        bwd_btn = QPushButton("BWD")
        bwd_btn.setCheckable(True)
        bwd_btn.setMinimumHeight(40)
        bwd_btn.clicked.connect(lambda: self.set_motor_state(motor_type, 2))
        
        # Create button group for exclusive selection
        btn_group = QButtonGroup(group)
        btn_group.addButton(fwd_btn, 1)
        btn_group.addButton(bwd_btn, 2)
        btn_group.setExclusive(True)
        
        layout.addWidget(fwd_btn)
        layout.addWidget(bwd_btn)
        group.setLayout(layout)
        
        # Store buttons for gamepad updates
        if motor_type == 'base':
            self.base_fwd_btn = fwd_btn
            self.base_bwd_btn = bwd_btn
        elif motor_type == 'shoulder':
            self.shoulder_fwd_btn = fwd_btn
            self.shoulder_bwd_btn = bwd_btn
        elif motor_type == 'elbow':
            self.elbow_fwd_btn = fwd_btn
            self.elbow_bwd_btn = bwd_btn
            
        return group

    def create_gripper_roller_control(self, name, control_type):
        group = QGroupBox(name)
        layout = QHBoxLayout()
        
        # Open button
        open_btn = QPushButton("Open")
        open_btn.setCheckable(True)
        open_btn.setMinimumHeight(40)
        open_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 2))
        
        # Close button
        close_btn = QPushButton("Close")
        close_btn.setCheckable(True)
        close_btn.setMinimumHeight(40)
        close_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 1))
        
        # Stop button
        stop_btn = QPushButton("Stop")
        stop_btn.setCheckable(True)
        stop_btn.setMinimumHeight(40)
        stop_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 0))
        
        # Create button group for exclusive selection
        btn_group = QButtonGroup(group)
        btn_group.addButton(open_btn, 2)
        btn_group.addButton(close_btn, 1)
        btn_group.addButton(stop_btn, 0)
        btn_group.setExclusive(True)
        stop_btn.setChecked(True)  # Default to stop
        
        layout.addWidget(open_btn)
        layout.addWidget(close_btn)
        layout.addWidget(stop_btn)
        group.setLayout(layout)
        
        # Store buttons for gamepad updates
        if control_type == 'gripper':
            self.gripper_open_btn = open_btn
            self.gripper_close_btn = close_btn
            self.gripper_stop_btn = stop_btn
        elif control_type == 'roller':
            self.roller_open_btn = open_btn
            self.roller_close_btn = close_btn
            self.roller_stop_btn = stop_btn
            
        return group

    def set_motor_state(self, motor_type, state):
        if motor_type == 'base':
            self.base_state = state
        elif motor_type == 'shoulder':
            self.shoulder_state = state
        elif motor_type == 'elbow':
            self.elbow_state = state

    def set_gripper_roller_state(self, control_type, state):
        if control_type == 'gripper':
            self.gripper_state = state
        elif control_type == 'roller':
            self.roller_state = state

    def get_direction_and_value(self, state):
        if state == 1:  # Forward
            return [1, self.shared_pwm]
        elif state == 2:  # Backward
            return [0, self.shared_pwm]
        else:  # Stop
            return [0, 0]

    def get_current_values(self):
        return [
            self.gripper_state,
            self.roller_state,
            self.servo_angle,
            self.get_direction_and_value(self.elbow_state),
            self.get_direction_and_value(self.shoulder_state),
            self.get_direction_and_value(self.base_state),
        ]

    def reset_all(self):
        # Reset motor states
        self.base_state = 0
        self.shoulder_state = 0
        self.elbow_state = 0
        
        # Reset gripper and roller states
        self.gripper_state = 0
        self.roller_state = 0
        
        # Reset servo angle
        self.servo_angle = 90
        
        # Reset PWM values
        self.shared_pwm = 0
        
        # Reset UI components
        for widget in self.findChildren(QSlider):
            if widget.minimum() == 0 and widget.maximum() == 180:  # Servo slider
                widget.setValue(90)
            else:  # PWM sliders
                widget.setValue(0)
        
        # Reset button groups
        for btn_group in self.findChildren(QButtonGroup):
            btn_group.setExclusive(False)
            for button in btn_group.buttons():
                button.setChecked(False)
            btn_group.setExclusive(True)
            
            # Set stop buttons to checked
            for button in btn_group.buttons():
                if btn_group.id(button) == 0:
                    button.setChecked(True)

    def update_output(self):
        values = self.get_current_values()
        
        # Update connection status
        if self.parent.ws_connected:
            self.status_label.setText("🟢 Connected")
            self.status_label.setStyleSheet("font-weight: bold; padding: 5px; color: green;")
        else:
            self.status_label.setText("🔴 Disconnected") 
            self.status_label.setStyleSheet("font-weight: bold; padding: 5px; color: red;")
        
        # Update gamepad status
        if self.parent.joystick:
            self.gamepad_status.setText("🟢 Gamepad connected")
            self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px; color: green;")
        else:
            self.gamepad_status.setText("🔴 No gamepad")
            self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px; color: red;")
        
        # Only send and update if values changed
        if values != self.last_values:
            print(f"ARM Values: {values}")
            
            # Send via WebSocket
            self.parent.send_websocket_message(values)
            
            # Update plot
            self.parent.update_arm_plot(values)
            self.last_values = values

    def cycle_motor_state(self, motor_type):
        """Cycle motor state: stop -> forward -> backward -> stop"""
        current_state = getattr(self, f"{motor_type}_state")
        new_state = (current_state + 1) % 3
        setattr(self, f"{motor_type}_state", new_state)
        
        # Update UI buttons
        if motor_type == 'base':
            if new_state == 1:
                self.base_fwd_btn.setChecked(True)
            elif new_state == 2:
                self.base_bwd_btn.setChecked(True)
            else:
                self.base_fwd_btn.setChecked(False)
                self.base_bwd_btn.setChecked(False)
                
        elif motor_type == 'shoulder':
            if new_state == 1:
                self.shoulder_fwd_btn.setChecked(True)
            elif new_state == 2:
                self.shoulder_bwd_btn.setChecked(True)
            else:
                self.shoulder_fwd_btn.setChecked(False)
                self.shoulder_bwd_btn.setChecked(False)
                
        elif motor_type == 'elbow':
            if new_state == 1:
                self.elbow_fwd_btn.setChecked(True)
            elif new_state == 2:
                self.elbow_bwd_btn.setChecked(True)
            else:
                self.elbow_fwd_btn.setChecked(False)
                self.elbow_bwd_btn.setChecked(False)

class WheelPublisherNode(Node):
    def __init__(self):
        super().__init__('wheel_publisher')
        self.wheel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        logger.info("Wheel Publisher Node initialized")
        self.active = True  # Track if node is active

    def wheel_drive(self, x: float, z: float, speed: float):
        if not self.active:
            return
            
        try:
            msg = Twist()
            msg.linear.x = x * speed
            msg.angular.z = z * speed
            self.wheel_pub.publish(msg)
            self.get_logger().info(f"WHEEL: x: {x:.2f} | z: {z:.2f} | speed: {speed:.2f}")
        except Exception as e:
            logger.error(f"Publishing error: {e}")

    def shutdown(self):
        self.active = False
        self.destroy_node()

class WheelControlGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setup_ui()
        
        # Control variables
        self.speed = 0.0
        self.x_dir = 0.0  # Forward/backward direction
        self.z_dir = 0.0  # Left/right steering
        self.button_states = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False
        }

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Status indicators
        status_layout = QHBoxLayout()
        self.ros_status = QLabel("🔴 ROS Disconnected")
        self.ros_status.setStyleSheet("font-weight: bold; padding: 5px;")
        self.gamepad_status = QLabel("🔴 No Gamepad")
        self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px;")
        status_layout.addWidget(self.ros_status)
        status_layout.addWidget(self.gamepad_status)
        layout.addLayout(status_layout)

        # Speed control
        speed_group = QGroupBox("Speed Control")
        speed_layout = QVBoxLayout()
        
        self.speed_label = QLabel("0%")
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.speed_label.setStyleSheet("font-weight: bold; color: #2b2d42;")
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(0)
        self.speed_slider.setMinimumHeight(35)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        # Style the slider
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background: #cccccc;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #2b2d42;
                width: 30px;
                height: 30px;
                margin: -10px 0;
                border-radius: 15px;
            }
        """)
        
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        # Direction buttons
        btn_group = QGroupBox("Direction Control")
        grid = QGridLayout()
        grid.setSpacing(15)
        grid.setContentsMargins(20, 20, 20, 20)
        
        # Create buttons
        self.btn_forward = QPushButton("↑ FORWARD")
        self.btn_backward = QPushButton("↓ BACKWARD")
        self.btn_left = QPushButton("← LEFT")
        self.btn_right = QPushButton("→ RIGHT")
        self.btn_stop = QPushButton("STOP")
        
        # Set button styles
        for btn in [self.btn_forward, self.btn_backward, self.btn_left, self.btn_right]:
            btn.setMinimumHeight(60)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2b2d42; 
                    color: white; 
                    border-radius: 10px; 
                    padding: 15px; 
                    font-weight: bold;
                    font-size: 16px;
                }
                QPushButton:pressed {
                    background-color: #4a4d6d;
                }
            """)
        
        self.btn_stop.setMinimumHeight(60)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; 
                color: white; 
                border-radius: 10px; 
                padding: 15px; 
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:pressed {
                background-color: #c0392b;
            }
        """)
        
        # Connect signals with press/release for toggle behavior
        self.btn_forward.pressed.connect(lambda: self.set_direction_button('forward', True))
        self.btn_forward.released.connect(lambda: self.set_direction_button('forward', False))
        self.btn_backward.pressed.connect(lambda: self.set_direction_button('backward', True))
        self.btn_backward.released.connect(lambda: self.set_direction_button('backward', False))
        self.btn_left.pressed.connect(lambda: self.set_direction_button('left', True))
        self.btn_left.released.connect(lambda: self.set_direction_button('left', False))
        self.btn_right.pressed.connect(lambda: self.set_direction_button('right', True))
        self.btn_right.released.connect(lambda: self.set_direction_button('right', False))
        self.btn_stop.clicked.connect(self.stop_all)
        
        # Layout
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)
        
        btn_group.setLayout(grid)
        btn_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 8px;
                margin-top: 1.5ex;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        layout.addWidget(btn_group)

    def set_direction_button(self, direction, pressed):
        """Set direction based on button press/release"""
        self.button_states[direction] = pressed
        self.update_direction_from_buttons()
        self.parent.update_wheel_plot()

    def update_direction_from_buttons(self):
        """Calculate direction from button states"""
        x = 0.0
        z = 0.0
        
        if self.button_states['forward']:
            x += 1.0
        if self.button_states['backward']:
            x -= 1.0
        if self.button_states['left']:
            z += 1.0
        if self.button_states['right']:
            z -= 1.0
        
        # Normalize if both directions pressed
        if x != 0 and z != 0:
            magnitude = np.sqrt(x**2 + z**2)
            x /= magnitude
            z /= magnitude
        
        self.x_dir = x
        self.z_dir = z

    def update_speed(self, value):
        """Handle speed slider changes"""
        self.speed = value / 100.0
        self.speed_label.setText(f"{value}%")

    def stop_all(self):
        """Stop all movement"""
        self.speed_slider.setValue(0)
        self.x_dir = 0.0
        self.z_dir = 0.0
        self.parent.update_wheel_plot()
        
        # Reset all button states
        for direction in self.button_states:
            self.button_states[direction] = False
        
        # Visually unpress all buttons
        self.btn_forward.setDown(False)
        self.btn_backward.setDown(False)
        self.btn_left.setDown(False)
        self.btn_right.setDown(False)

    def update_output(self):
        """Publish control values to ROS and update UI"""
        # Update status indicators
        ros_connected = self.parent.ros_node is not None and self.parent.ros_node.active
        gamepad_connected = self.parent.joystick is not None
        
        self.ros_status.setText("🟢 ROS Connected" if ros_connected else "🔴 ROS Disconnected")
        self.ros_status.setStyleSheet(f"color: {'green' if ros_connected else 'red'}; font-weight: bold;")
        
        self.gamepad_status.setText("🟢 Gamepad Connected" if gamepad_connected else "🔴 No Gamepad")
        self.gamepad_status.setStyleSheet(f"color: {'green' if gamepad_connected else 'red'}; font-weight: bold;")
        
        # Publish to ROS
        if ros_connected:
            try:
                self.parent.ros_node.wheel_drive(
                    self.x_dir,
                    self.z_dir,
                    self.speed
                )
            except Exception as e:
                logger.error(f"Error publishing to ROS: {e}")

class CombinedControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mars Rover Control System")
        self.setGeometry(100, 100, 1600, 900)
        
        # Initialize pygame for gamepad support
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.init_gamepad()

        # WebSocket client setup
        self.ws = None
        self.ws_connected = False
        self.shutting_down = False
        self.setup_websocket_client()

        # ROS2 setup
        self.ros_node = None
        self.setup_ros_node()

        # GUI setup
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)

        # Create splitter for the three sections
        splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(splitter)

        # Left section: Arm Control
        self.arm_control = ArmControlGUI(self)
        splitter.addWidget(self.arm_control)

        # Middle section: Wheel Control
        self.wheel_control = WheelControlGUI(self)
        splitter.addWidget(self.wheel_control)

        # Right section: Plots
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        plot_layout.setSpacing(10)
        
        # Arm plot
        self.arm_figure = Figure(figsize=(6, 6))
        self.arm_canvas = FigureCanvas(self.arm_figure)
        self.arm_ax = self.arm_figure.add_subplot(111, projection='3d')
        self.arm_ax.set_xlim(-30, 30)
        self.arm_ax.set_ylim(-30, 30)
        self.arm_ax.set_zlim(0, 50)
        self.arm_ax.set_xlabel("X")
        self.arm_ax.set_ylabel("Y")
        self.arm_ax.set_zlabel("Z")
        self.arm_ax.set_title("Arm Position", fontsize=10)
        self.arm_ax.grid(True)
        
        arm_plot_group = QGroupBox("Arm Visualization")
        arm_plot_layout = QVBoxLayout()
        arm_plot_layout.addWidget(self.arm_canvas)
        arm_plot_group.setLayout(arm_plot_layout)
        plot_layout.addWidget(arm_plot_group, 1)
        
        # Wheel plot
        self.wheel_figure = Figure(figsize=(6, 6))
        self.wheel_canvas = FigureCanvas(self.wheel_figure)
        self.wheel_ax = self.wheel_figure.add_subplot(111)
        self.wheel_ax.set_xlim(-1.5, 1.5)
        self.wheel_ax.set_ylim(-1.5, 1.5)
        self.wheel_ax.grid(True, linestyle='--', alpha=0.7)
        self.wheel_ax.set_title("Wheel Direction", fontsize=10)
        self.wheel_ax.set_xlabel("Left/Right", fontsize=8)
        self.wheel_ax.set_ylabel("Forward/Backward", fontsize=8)
        
        # Add center point and direction arrow
        self.center_circle = Circle((0, 0), 0.05, color='#3498db', zorder=5)
        self.wheel_ax.add_patch(self.center_circle)
        self.direction_arrow = None
        
        # Add axis labels
        self.wheel_ax.text(1.4, 0.05, "→", fontsize=16, ha='center', va='center')
        self.wheel_ax.text(0.05, 1.4, "↑", fontsize=16, ha='center', va='center')
        self.wheel_ax.text(0, -1.4, "BACKWARD", ha='center', va='center', fontsize=8)
        self.wheel_ax.text(0, 1.4, "FORWARD", ha='center', va='center', fontsize=8)
        self.wheel_ax.text(-1.4, 0, "LEFT", ha='center', va='center', fontsize=8, rotation=90)
        self.wheel_ax.text(1.4, 0, "RIGHT", ha='center', va='center', fontsize=8, rotation=-90)
        
        wheel_plot_group = QGroupBox("Wheel Visualization")
        wheel_plot_layout = QVBoxLayout()
        wheel_plot_layout.addWidget(self.wheel_canvas)
        wheel_plot_group.setLayout(wheel_plot_layout)
        plot_layout.addWidget(wheel_plot_group, 1)
        
        splitter.addWidget(plot_widget)

        # Set splitter sizes
        splitter.setSizes([400, 400, 800])

        # Timers
        self.arm_timer = QTimer()
        self.arm_timer.timeout.connect(self.arm_control.update_output)
        self.arm_timer.start(50)  # 20Hz update
        
        self.wheel_timer = QTimer()
        self.wheel_timer.timeout.connect(self.wheel_control.update_output)
        self.wheel_timer.start(50)  # 20Hz update
        
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.update_gamepad)
        self.gamepad_timer.start(100)  # 10Hz update

    def init_gamepad(self):
        """Initialize gamepad if available"""
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                logger.info(f"Gamepad connected: {self.joystick.get_name()}")
            except Exception as e:
                logger.error(f"Gamepad init error: {e}")
                self.joystick = None
        else:
            logger.warning("No gamepad detected")
            self.joystick = None

    def setup_websocket_client(self):
        """Setup WebSocket client to connect to the main server"""
        def on_message(ws, message):
            try:
                if message.startswith('[') or message.startswith('{'):
                    data = json.loads(message)
                    logger.info(f"Received from server: {data}")
                else:
                    logger.info(f"Received: {message}")
            except json.JSONDecodeError:
                logger.info(f"Received text: {message}")

        def on_open(ws):
            logger.info("✅ Connected to WebSocket server")
            self.ws_connected = True
            ws.send("ARM GUI Connected!")

        def on_error(ws, error):
            logger.error(f"WebSocket error: {error}")
            self.ws_connected = False

        def on_close(ws, close_status_code, close_msg):
            logger.info("Disconnected from WebSocket server")
            self.ws_connected = False

        def connect_websocket():
            while True:
                try:
                    logger.info("Attempting to connect to WebSocket server...")
                    self.ws = websocket.WebSocketApp(
                        "ws://192.168.0.101:8765",  # Connect to main server
                        on_open=on_open,
                        on_message=on_message,
                        on_error=on_error,
                        on_close=on_close
                    )
                    self.ws.run_forever()
                    
                    # If we get here, connection was closed
                    if self.shutting_down:
                        break
                        
                    logger.info("Attempting to reconnect in 5 seconds...")
                    threading.Event().wait(5)  # Sleep for 5 seconds
                    
                except Exception as e:
                    logger.error(f"Connection error: {e}")
                    threading.Event().wait(5)

        # Start WebSocket client in background thread
        self.ws_thread = threading.Thread(target=connect_websocket, daemon=True)
        self.ws_thread.start()

    def send_websocket_message(self, message):
        """Send message through WebSocket if connected"""
        if self.ws_connected and self.ws:
            try:
                if isinstance(message, (list, dict)):
                    message = json.dumps(message)
                self.ws.send(message)
                return True
            except Exception as e:
                logger.error(f"Failed to send message: {e}")
                self.ws_connected = False
        return False

    def setup_ros_node(self):
        """Setup ROS2 node in a separate thread"""
        def ros_thread_function():
            try:
                rclpy.init()
                self.ros_node = WheelPublisherNode()
                logger.info("ROS2 node initialized")
                
                # Spin until shutdown
                while rclpy.ok() and not self.shutting_down:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                
                if self.ros_node:
                    self.ros_node.shutdown()
                rclpy.shutdown()
            except Exception as e:
                logger.error(f"ROS node error: {e}")

        self.ros_thread = threading.Thread(target=ros_thread_function, daemon=True)
        self.ros_thread.start()

    def update_arm_plot(self, values):
        _, _, wrist, elbow, shoulder, base = values
        base_angle = (base[0]*2 - 1) * (base[1] / 1023) * 90
        shoulder_angle = (shoulder[0]*2 - 1) * (shoulder[1] / 1023) * 90
        elbow_angle = (elbow[0]*2 - 1) * (elbow[1] / 1023) * 90

        x0, y0, z0 = 0, 0, 0
        x1 = L1 * np.cos(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle))
        y1 = L1 * np.sin(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle))
        z1 = L1 * np.sin(np.radians(shoulder_angle))

        x2 = x1 + L2 * np.cos(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle + elbow_angle))
        y2 = y1 + L2 * np.sin(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle + elbow_angle))
        z2 = z1 + L2 * np.sin(np.radians(shoulder_angle + elbow_angle))

        x3 = x2 + L3 * np.cos(np.radians(base_angle)) * np.cos(np.radians(wrist))
        y3 = y2 + L3 * np.sin(np.radians(base_angle)) * np.cos(np.radians(wrist))
        z3 = z2 + L3 * np.sin(np.radians(wrist))

        self.arm_ax.cla()
        self.arm_ax.plot([x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3], 
                    color='#3f72af', marker='o', linewidth=3, markersize=8)
        
        # Add joint labels
        self.arm_ax.text(x0, y0, z0, 'Base', fontsize=8)
        self.arm_ax.text(x1, y1, z1, 'Shoulder', fontsize=8)
        self.arm_ax.text(x2, y2, z2, 'Elbow', fontsize=8)
        self.arm_ax.text(x3, y3, z3, 'Wrist', fontsize=8)
        
        # Set limits and labels
        self.arm_ax.set_xlim(-30, 30)
        self.arm_ax.set_ylim(-30, 30)
        self.arm_ax.set_zlim(0, 50)
        self.arm_ax.set_xlabel("X")
        self.arm_ax.set_ylabel("Y")
        self.arm_ax.set_zlabel("Z")
        self.arm_ax.set_title("Arm Position")
        self.arm_ax.grid(True)
        self.arm_canvas.draw()

    def update_wheel_plot(self):
        """Update the wheel direction visualization"""
        # Clear previous arrow if it exists
        if self.direction_arrow:
            try:
                self.direction_arrow.remove()
            except ValueError:
                # Arrow was already removed, ignore error
                pass
            self.direction_arrow = None
        
        # Create new arrow if needed
        if self.wheel_control.x_dir != 0 or self.wheel_control.z_dir != 0:
            # Arrow points: (x=left/right, y=forward/backward)
            dx = self.wheel_control.z_dir  # Left/Right
            dy = self.wheel_control.x_dir  # Forward/Backward
            
            # Create a new arrow
            self.direction_arrow = Arrow(0, 0, dx, dy, width=0.2, color='#e74c3c', zorder=4)
            self.wheel_ax.add_patch(self.direction_arrow)
            
            # Add magnitude indicator
            magnitude = np.sqrt(dx**2 + dy**2)
            if magnitude > 0.1:
                self.wheel_ax.text(dx/2, dy/2, f"{magnitude*100:.0f}%", 
                            fontsize=10, ha='center', va='center',
                            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
        
        self.wheel_canvas.draw()

    def toggle_arm_plot_detach(self):
        if not hasattr(self, 'arm_detached_window') or not self.arm_detached_window:
            # Create detached window
            self.arm_detached_window = QMainWindow()
            self.arm_detached_window.setWindowTitle("Arm Plot")
            self.arm_detached_window.setGeometry(300, 300, 600, 600)
            self.arm_detached_window.setCentralWidget(self.arm_canvas)
            self.arm_detached_window.show()
            self.arm_control.detach_btn.setText("Attach Plot")
        else:
            self.arm_detached_window.close()
            self.arm_detached_window = None
            # Re-add to layout
            for i in reversed(range(self.arm_control.layout().count())):
                self.arm_control.layout().itemAt(i).widget().setParent(None)
            self.arm_control.layout().addWidget(self.arm_canvas)
            self.arm_control.detach_btn.setText("Detach Plot")

    def update_gamepad(self):
        """Process gamepad inputs for both arm and wheel controls"""
        if not self.joystick:
            # Try to reconnect if gamepad wasn't detected at startup
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                logger.info(f"Gamepad connected: {self.joystick.get_name()}")
            return
            
        # Process pygame events
        for event in pygame.event.get():
            pass  # We're using state polling instead of events
        
        try:
            # Get current button states
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            hats = self.joystick.get_numhats()
            hat_state = (0, 0)
            if hats > 0:
                hat_state = self.joystick.get_hat(0)  # D-pad state
            
            # Xbox controller mappings:
            # Buttons: [Y, B, A, X, LB, RB, LT, RT, BACK, START, L, R]
            # Indices:  0  1  2  3  4   5   6   7    8      9     10 11
            
            # Continuous adjustment for triggers
            current_time = pygame.time.get_ticks()
            
            # ARM CONTROL
            # Shared PWM control with RT/LT
            if buttons[7]:  # RT (button 7) - increase shared PWM
                new_pwm = min(1023, self.arm_control.shared_pwm + 10)
                if new_pwm != self.arm_control.shared_pwm:
                    self.arm_control.shared_pwm = new_pwm
                    self.arm_control.shared_pwm_slider_ref.setValue(self.arm_control.shared_pwm)
                    self.arm_control.shared_pwm_label.setText(str(self.arm_control.shared_pwm))
                    
            if buttons[6]:  # LT (button 6) - decrease shared PWM
                new_pwm = max(0, self.arm_control.shared_pwm - 10)
                if new_pwm != self.arm_control.shared_pwm:
                    self.arm_control.shared_pwm = new_pwm
                    self.arm_control.shared_pwm_slider_ref.setValue(self.arm_control.shared_pwm)
                    self.arm_control.shared_pwm_label.setText(str(self.arm_control.shared_pwm))
            
            # Wrist servo control with RB/LB
            if buttons[5]:  # RB (button 5) - increase wrist servo
                new_angle = min(180, self.arm_control.servo_angle + 1)
                if new_angle != self.arm_control.servo_angle:
                    self.arm_control.servo_angle = new_angle
                    self.arm_control.servo_slider_ref.setValue(self.arm_control.servo_angle)
                    self.arm_control.servo_label.setText(str(self.arm_control.servo_angle))
                    
            if buttons[4]:  # LB (button 4) - decrease wrist servo
                new_angle = max(0, self.arm_control.servo_angle - 1)
                if new_angle != self.arm_control.servo_angle:
                    self.arm_control.servo_angle = new_angle
                    self.arm_control.servo_slider_ref.setValue(self.arm_control.servo_angle)
                    self.arm_control.servo_label.setText(str(self.arm_control.servo_angle))
            
            # Motor controls (B, Y, X) - toggle on press
            if buttons[1] and not self.arm_control.prev_buttons[1]:  # B button (index 1) - base motor
                self.arm_control.cycle_motor_state('base')
                
            if buttons[0] and not self.arm_control.prev_buttons[0]:  # Y button (index 0) - shoulder motor
                self.arm_control.cycle_motor_state('shoulder')
                
            if buttons[3] and not self.arm_control.prev_buttons[3]:  # X button (index 3) - elbow motor
                self.arm_control.cycle_motor_state('elbow')
            
            # Gripper controls
            if hat_state[1] == 1:  # D-pad up
                self.arm_control.set_gripper_roller_state('gripper', 2)  # Open
                self.arm_control.gripper_open_btn.setChecked(True)
            elif hat_state[1] == -1:  # D-pad down
                self.arm_control.set_gripper_roller_state('gripper', 1)  # Close
                self.arm_control.gripper_close_btn.setChecked(True)
                
            if buttons[10] and not self.arm_control.prev_buttons[10]:  # L button (index 10)
                self.arm_control.set_gripper_roller_state('gripper', 0)  # Stop
                self.arm_control.gripper_stop_btn.setChecked(True)
            
            # Roller controls
            if hat_state[0] == 1:  # D-pad right
                self.arm_control.set_gripper_roller_state('roller', 2)  # Open
                self.arm_control.roller_open_btn.setChecked(True)
            elif hat_state[0] == -1:  # D-pad left
                self.arm_control.set_gripper_roller_state('roller', 1)  # Close
                self.arm_control.roller_close_btn.setChecked(True)
                
            if buttons[11] and not self.arm_control.prev_buttons[11]:  # R button (index 11)
                self.arm_control.set_gripper_roller_state('roller', 0)  # Stop
                self.arm_control.roller_stop_btn.setChecked(True)
            
            # Reset button (START)
            if buttons[9] and not self.arm_control.prev_buttons[9]:
                self.arm_control.reset_all()
            
            # Save current button states for next comparison
            self.arm_control.prev_buttons = buttons
            
            # WHEEL CONTROL
            # Get axis values safely
            num_axes = self.joystick.get_numaxes()
            left_x = 0.0
            left_y = 0.0
            right_x = 0.0  # Right stick X for speed control

            if num_axes > 3:
                right_x = self.joystick.get_axis(3)
            if num_axes > 2:
                left_x = self.joystick.get_axis(2)
            
            if num_axes > 0:
                left_x = self.joystick.get_axis(0)
            if num_axes > 1:
                left_y = self.joystick.get_axis(1)
            
            # Calculate speed from right stick
            current_slider_val = self.wheel_control.speed_slider.value()
            change = int(right_x * 5)  # Adjust sensitivity as needed
            new_speed_val = max(0, min(100, current_slider_val + change))
            self.wheel_control.speed_slider.setValue(new_speed_val)
            self.wheel_control.speed = new_speed_val / 100.0
            
            # Calculate direction from left stick
            deadzone = 0.2
            self.wheel_control.x_dir = -left_y if abs(left_y) > deadzone else 0.0
            self.wheel_control.z_dir = left_x if abs(left_x) > deadzone else 0.0
            
            # Normalize direction vector
            magnitude = np.sqrt(self.wheel_control.x_dir**2 + self.wheel_control.z_dir**2)
            if magnitude > 1.0:
                self.wheel_control.x_dir /= magnitude
                self.wheel_control.z_dir /= magnitude
                
            # START button to stop
            if self.joystick.get_button(9):  # START button
                self.wheel_control.stop_all()
                
            # Update plot
            self.update_wheel_plot()
            
        except Exception as e:
            logger.error(f"Gamepad error: {e}")
            self.joystick = None

    def closeEvent(self, event):
        """Clean shutdown when GUI is closed"""
        logger.info("Shutting down Combined Control GUI...")
        self.shutting_down = True
        
        # Stop timers
        self.arm_timer.stop()
        self.wheel_timer.stop()
        self.gamepad_timer.stop()
        
        # Shutdown WebSocket
        if self.ws_connected and self.ws:
            try:
                self.ws.send("ARM GUI Disconnecting...")
                self.ws.close()
            except:
                pass
        
        # Shutdown ROS node
        if self.ros_node:
            self.ros_node.shutdown()
            
        # Clean up pygame
        pygame.quit()
        
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = CombinedControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()