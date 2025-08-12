import sys
import socket
import numpy as np
import open3d as o3d
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, 
    QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QTextEdit, QMessageBox, QFileDialog,
    QComboBox
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QFont

class RobotArmClient(QThread):
    arm_status = pyqtSignal(str)
    scanner_status = pyqtSignal(str)
    scan_complete = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.arm_socket = None
        self.scanner_socket = None
        self.arm_connected = False
        self.scanner_connected = False

    def connect_arm(self, host, port):
        try:
            self.arm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.arm_socket.settimeout(5)
            self.arm_socket.connect((host, port))
            self.arm_connected = True
            self.arm_status.emit(f"Connected to arm at {host}:{port}")
            return True
        except Exception as e:
            self.arm_status.emit(f"Arm connection error: {str(e)}")
            return False

    def connect_scanner(self, host, port):
        try:
            self.scanner_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.scanner_socket.settimeout(5)
            self.scanner_socket.connect((host, port))
            self.scanner_connected = True
            self.scanner_status.emit(f"Connected to scanner at {host}:{port}")
            return True
        except Exception as e:
            self.scanner_status.emit(f"Scanner connection error: {str(e)}")
            return False

    def send_arm_command(self, command):
        if not self.arm_connected:
            self.arm_status.emit("Not connected to arm")
            return None

        try:
            self.arm_socket.settimeout(5)
            self.arm_socket.sendall(command.encode('utf-8'))
            response = self.arm_socket.recv(1024).decode('utf-8')
            self.arm_status.emit(f"Arm response: {response}")
            return response
        except socket.timeout:
            self.arm_status.emit("Arm socket timed out")
            return None
        except Exception as e:
            self.arm_status.emit(f"Arm command error: {str(e)}")
            return None


    def send_scanner_command(self, command):
        if not self.scanner_connected:
            self.scanner_status.emit("Not connected to scanner")
            return None

        try:
            self.scanner_socket.sendall(command.encode('utf-8'))
            response = self.scanner_socket.recv(4096).decode('utf-8')
            self.scanner_status.emit(f"Scanner response: {response}")
            return response
        except Exception as e:
            self.scanner_status.emit(f"Scanner command error: {str(e)}")
            return None

    def move_to_position(self, pos_name):
        return self.send_arm_command(f"MOVE {pos_name}")
 

    def capture_scan(self):
        response = self.send_scanner_command("CAPTURE")
        if response == "SCAN_COMPLETE":
            point_cloud = np.random.rand(1000, 3)  # Mock data
            self.scan_complete.emit(point_cloud)
            return True
        return False

    def disconnect_arm(self):
        if self.arm_socket:
            try:
                self.arm_socket.sendall("DISCONNECT".encode('utf-8'))
                # Wait for a response from the arm (e.g., confirmation)
                self.arm_socket.settimeout(3)  # Optional: timeout to prevent hanging
                response = self.arm_socket.recv(1024).decode('utf-8')
                self.arm_status.emit(f"Arm disconnect response: {response}")

            except Exception as e:
                self.arm_status.emit(f"Error sending disconnect command: {str(e)}")
            self.arm_socket.close()
            self.arm_connected = False
            self.arm_status.emit("Disconnected from arm")


    def disconnect_scanner(self):
        if self.scanner_socket:
            self.scanner_socket.close()
            self.scanner_connected = False
            self.scanner_status.emit("Disconnected from scanner")

    def disconnect_all(self):
        self.disconnect_arm()
        self.disconnect_scanner()

class PointCloudViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.point_cloud = None

    def init_ui(self):
        layout = QVBoxLayout()
        self.view_button = QPushButton("View Point Cloud")
        self.view_button.clicked.connect(self.show_point_cloud)
        layout.addWidget(self.view_button)
        self.setLayout(layout)

    def show_point_cloud(self):
        if self.point_cloud is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
            o3d.visualization.draw_geometries([pcd])

    def update_point_cloud(self, data):
        self.point_cloud = data

class RoboticArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm & Scanner Control System")
        self.setGeometry(100, 100, 600, 500)
        self.arm_client = RobotArmClient()
        self.scans = []
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left Panel (Connection and Control)
        left_panel = QVBoxLayout()

        # Arm Connection Group
        arm_conn_group = QGroupBox("Arm Connection")
        arm_conn_layout = QVBoxLayout()
        
        self.arm_host_input = QLineEdit("127.0.0.1")
        self.arm_port_input = QLineEdit("12345")
        self.arm_connect_btn = QPushButton("Connect Arm")
        self.arm_disconnect_btn = QPushButton("Disconnect Arm")
        self.arm_disconnect_btn.setEnabled(False)
        
        arm_conn_layout.addWidget(QLabel("Arm Controller IP:"))
        arm_conn_layout.addWidget(self.arm_host_input)
        arm_conn_layout.addWidget(QLabel("Port:"))
        arm_conn_layout.addWidget(self.arm_port_input)
        
        arm_btn_layout = QHBoxLayout()
        arm_btn_layout.addWidget(self.arm_connect_btn)
        arm_btn_layout.addWidget(self.arm_disconnect_btn)
        arm_conn_layout.addLayout(arm_btn_layout)
        
        arm_conn_group.setLayout(arm_conn_layout)

        # Scanner Connection Group
        scanner_conn_group = QGroupBox("Scanner Connection")
        scanner_conn_layout = QVBoxLayout()
        
        self.scanner_host_input = QLineEdit("127.0.0.1")
        self.scanner_port_input = QLineEdit("54321")
        self.scanner_connect_btn = QPushButton("Connect Scanner")
        self.scanner_disconnect_btn = QPushButton("Disconnect Scanner")
        self.scanner_disconnect_btn.setEnabled(False)
        
        scanner_conn_layout.addWidget(QLabel("Scanner IP:"))
        scanner_conn_layout.addWidget(self.scanner_host_input)
        scanner_conn_layout.addWidget(QLabel("Port:"))
        scanner_conn_layout.addWidget(self.scanner_port_input)
        
        scanner_btn_layout = QHBoxLayout()
        scanner_btn_layout.addWidget(self.scanner_connect_btn)
        scanner_btn_layout.addWidget(self.scanner_disconnect_btn)
        scanner_conn_layout.addLayout(scanner_btn_layout)
        
        scanner_conn_group.setLayout(scanner_conn_layout)

        # Arm Control Group
        control_group = QGroupBox("Arm Control")
        control_layout = QVBoxLayout()
        
        self.pos_combo = QComboBox()
        self.pos_combo.addItems(["Initial Position", "Scan Position 1", "Scan Position 2", "Scan Position 3"])
        self.move_btn = QPushButton("Move to Position")
        self.move_btn.setEnabled(False)
        
        self.capture_btn = QPushButton("Capture Scan")
        self.capture_btn.setEnabled(False)
        
        control_layout.addWidget(QLabel("Select Position:"))
        control_layout.addWidget(self.pos_combo)
        control_layout.addWidget(self.move_btn)
        control_layout.addWidget(self.capture_btn)
        control_group.setLayout(control_layout)

        # Add to left panel
        left_panel.addWidget(arm_conn_group)
        left_panel.addWidget(scanner_conn_group)
        left_panel.addWidget(control_group)
        left_panel.addStretch(1)

        # Right Panel (Visualization and Status)
        right_panel = QVBoxLayout()

        # Point Cloud Group
        pc_group = QGroupBox("Point Cloud")
        pc_layout = QVBoxLayout()
        
        self.pc_viewer = PointCloudViewer()
        self.save_btn = QPushButton("Save Point Cloud")
        self.save_btn.setEnabled(False)
        
        pc_layout.addWidget(self.pc_viewer)
        pc_layout.addWidget(self.save_btn)
        pc_group.setLayout(pc_layout)

        # Status Group
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.status_display.setFont(QFont("Courier New", 10))
        
        status_layout.addWidget(self.status_display)
        status_group.setLayout(status_layout)

        # Add to right panel
        right_panel.addWidget(pc_group)
        right_panel.addWidget(status_group)

        # Assemble main layout
        main_layout.addLayout(left_panel, stretch=1)
        main_layout.addLayout(right_panel, stretch=2)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def setup_connections(self):
        self.arm_connect_btn.clicked.connect(self.connect_arm)
        self.arm_disconnect_btn.clicked.connect(self.disconnect_arm)
        self.scanner_connect_btn.clicked.connect(self.connect_scanner)
        self.scanner_disconnect_btn.clicked.connect(self.disconnect_scanner)
        self.move_btn.clicked.connect(self.move_arm)
        self.capture_btn.clicked.connect(self.capture_scan)
        self.save_btn.clicked.connect(self.save_point_cloud)
        
        self.arm_client.arm_status.connect(self.update_status)
        self.arm_client.scanner_status.connect(self.update_status)
        self.arm_client.scan_complete.connect(self.handle_scan_data)

    def connect_arm(self):
        host = self.arm_host_input.text().strip()
        port = self.arm_port_input.text().strip()

        if not host or not port:
            QMessageBox.warning(self, "Input Error", "Please enter arm host and port")
            return

        try:
            port = int(port)
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Invalid arm port number")
            return

        if self.arm_client.connect_arm(host, port):
            self.arm_connect_btn.setEnabled(False)
            self.arm_disconnect_btn.setEnabled(True)
            self.move_btn.setEnabled(True)

    def connect_scanner(self):
        host = self.scanner_host_input.text().strip()
        port = self.scanner_port_input.text().strip()

        if not host or not port:
            QMessageBox.warning(self, "Input Error", "Please enter scanner host and port")
            return

        try:
            port = int(port)
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Invalid scanner port number")
            return

        if self.arm_client.connect_scanner(host, port):
            self.scanner_connect_btn.setEnabled(False)
            self.scanner_disconnect_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)
            self.save_btn.setEnabled(True)

    def disconnect_arm(self):
        self.arm_client.disconnect_arm()
        self.arm_connect_btn.setEnabled(True)
        self.arm_disconnect_btn.setEnabled(False)
        self.move_btn.setEnabled(False)

    def disconnect_scanner(self):
        self.arm_client.disconnect_scanner()
        self.scanner_connect_btn.setEnabled(True)
        self.scanner_disconnect_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        self.save_btn.setEnabled(False)

    def move_arm(self):
        pos = self.pos_combo.currentText()
        self.arm_client.move_to_position(pos)

    def capture_scan(self):
        if self.arm_client.capture_scan():
            self.status_display.append("Scan completed successfully")
        else:
            self.status_display.append("Scan failed")

    def handle_scan_data(self, point_cloud):
        self.scans.append(point_cloud)
        self.pc_viewer.update_point_cloud(point_cloud)
        self.status_display.append("Point cloud data received")

    def save_point_cloud(self):
        if not self.scans:
            self.status_display.append("No scans to save")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Point Cloud", "", "PLY Files (*.ply);;All Files (*)"
        )
        
        if file_path:
            combined_cloud = np.vstack(self.scans)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(combined_cloud)
            o3d.io.write_point_cloud(file_path, pcd)
            self.status_display.append(f"Saved point cloud to {file_path}")

    def update_status(self, message):
        self.status_display.append(message)

    def closeEvent(self, event):
        self.arm_client.disconnect_all()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoboticArmGUI()
    window.show()
    sys.exit(app.exec())