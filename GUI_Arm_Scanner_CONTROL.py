import os # for filename manipulation
import sys
import math
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
            self.arm_socket.settimeout(20) # wait until 20s for a response else shortcut the process
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
            self.scanner_socket.settimeout(20)  # wait until 20s for a response else shortcut the process
            self.scanner_socket.sendall(command.encode('utf-8'))
            response = self.scanner_socket.recv(1024).decode('utf-8')
            self.scanner_status.emit(f"Scanner response: {response}")
            return response
        except socket.timeout:
            self.scanner_status.emit("Scanner socket timed out")
            return None
        except Exception as e:
            self.scanner_status.emit(f"Scanner command error: {str(e)}")
            return None

    def move_to_position(self, pos_name):
        return self.send_arm_command(f"MOVE {pos_name}")
 
    def capture_scan(self):
        response = self.send_scanner_command("CAPTURE")
        if response == "SCAN_COMPLETED":
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
            try:
                self.scanner_socket.sendall("DISCONNECT".encode('utf-8'))
                # Wait for a response from the scanner (e.g., confirmation)
                self.scanner_socket.settimeout(3)  # Optional: timeout to prevent hanging
                response = self.scanner_socket.recv(1024).decode('utf-8')
                self.scanner_status.emit(f"Scanner disconnect response: {response}")

            except Exception as e:
                self.scanner_status.emit(f"Error sending disconnect command: {str(e)}")
                
            self.scanner_socket.close()
            self.scanner_connected = False
            self.scanner_status.emit("Disconnected from scanner")

    def disconnect_all(self):
        self.disconnect_arm()
        self.disconnect_scanner()

class PointCloudViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.point_cloud = None  # This stores numpy array points
        self.o3d_pointcloud = None  # This will store the Open3D point cloud object

    def update_point_cloud(self, data):
        """Store both numpy and Open3D versions of the point cloud"""
        self.point_cloud = data
        self.o3d_pointcloud = o3d.geometry.PointCloud()
        self.o3d_pointcloud.points = o3d.utility.Vector3dVector(data)

    def show_point_cloud(self):
        """Visualize the point cloud using Open3D"""
        if self.o3d_pointcloud is not None:
            # Customize visualization
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name="Point Cloud Viewer")
            vis.add_geometry(self.o3d_pointcloud)
            
            # Set visualization options
            opt = vis.get_render_option()
            opt.background_color = np.array([0.1, 0.1, 0.1])  # Dark background
            opt.point_size = 2.0
            
            vis.run()
            vis.destroy_window()


class RoboticArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm & Scanner Control System")
        self.setGeometry(100, 100, 700, 600)
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

        self.open_pcl_btn = QPushButton("No file selected")

        self.view_btn = QPushButton("View Point Cloud")
        self.view_btn.setEnabled(False)
        
        pc_layout.addWidget(self.pc_viewer)
        pc_layout.addWidget(self.save_btn)
        pc_layout.addWidget(QLabel("PCL to view:"))
        pc_layout.addWidget(self.open_pcl_btn)
        pc_layout.addWidget(self.view_btn)
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
        # Arm connection
        self.arm_connect_btn.clicked.connect(self.connect_arm)
        self.arm_disconnect_btn.clicked.connect(self.disconnect_arm)

        # Scanner connection
        self.scanner_connect_btn.clicked.connect(self.connect_scanner)
        self.scanner_disconnect_btn.clicked.connect(self.disconnect_scanner)

        # Arm Control 
        self.move_btn.clicked.connect(self.move_arm)
        self.capture_btn.clicked.connect(self.capture_scan)
        
        # Point Cloud
        self.save_btn.clicked.connect(self.save_point_cloud)
        self.view_btn.clicked.connect(self.show_point_cloud)
        self.open_pcl_btn.clicked.connect(self.open_pcl_dialog)
        
        # Status
        self.arm_client.arm_status.connect(self.update_status)
        self.arm_client.scanner_status.connect(self.update_status)

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
        if not self.arm_client.capture_scan():
            self.update_status("Scan failed")

    def save_point_cloud(self):
        try:
            file_path, _ = QFileDialog.getSaveFileName(
                self,
                "Save Point Cloud",
                "pcl.ply",  # Default name with extension
                "All Files (*)"
            )

            if not file_path:  # User cancelled
                return

            # Ensure .ply extension
            file_path = os.path.splitext(file_path)[0] + '.ply'

            self.update_status(f"Attempting to save point cloud to {file_path}...")

            # Use the scanner client to send the command
            response = self.arm_client.send_scanner_command(f"SAVE:{file_path}")

            if response:
                self.view_btn.setEnabled(True)  # Fixed typo from view_button to view_btn
            else:
                self.status_display.append("Failed to receive confirmation from scanner")

        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save point cloud:\n{str(e)}")
            self.status_display.append(f"Save error: {str(e)}")

        finally:
            # Enabling view PCL button
            self.view_btn.setEnabled(True)

    def open_pcl_dialog(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select a File",
            "",
            "Point Cloud Files (*.xyz *.xyzn *.xyzrgb *.pts *.ply *.pcd);; ASC Files (*.asc);; All Files (*)"
        )
        
        if file_path:
            try:
                filename = os.path.basename(file_path)
                self.open_pcl_btn.setText(f"Selected: {filename}")
                
                # Load file (ASC or standard format)
                if file_path.lower().endswith('.asc'):
                    self.point_cloud = self.read_asc_file(file_path)
                else:
                    self.point_cloud = o3d.io.read_point_cloud(file_path)
                
                if not self.point_cloud.has_points():
                    raise ValueError("The file contains no points")
                
                # Update the viewer with the point cloud data
                self.pc_viewer.update_point_cloud(np.asarray(self.point_cloud.points))
                
                self.status_display.append(f"Loaded: {filename}")
                self.view_btn.setEnabled(True)  # Enable view button
                
                # Display point cloud info
                self.status_display.append(f"Points: {len(self.point_cloud.points)}")
                if self.point_cloud.has_colors():
                    self.status_display.append("Contains color information")
                if self.point_cloud.has_normals():
                    self.status_display.append("Contains normal vectors")
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file:\n{str(e)}")
                self.point_cloud = None  # Clear on error

    def read_asc_file(self, file_path):
        points = []
        colors = []
        normals = []
        is_rgb = None
        
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                    
                parts = line.split()
                try:
                    # Read XYZ (first 3 columns)
                    x, y, z = map(float, parts[:3])
                    points.append([x, y, z])
                    
                    if len(parts) >= 6:
                        col4, col5, col6 = map(float, parts[3:6])
                        
                        # Auto-detection
                        if is_rgb is None:
                            # Check if values look like normals (unit vectors)
                            magnitude = math.sqrt(col4**2 + col5**2 + col6**2)
                            is_normal = (0.9 <= magnitude <= 1.1)  # Allow 10% tolerance
                            
                            # Check if values look like RGB (0-255)
                            is_rgb = (not is_normal and 
                                    0 <= col4 <= 255 and 
                                    0 <= col5 <= 255 and 
                                    0 <= col6 <= 255)
                            
                            # If neither, default to normals
                            if not is_rgb and not is_normal:
                                is_rgb = False
                        
                        if is_rgb:
                            colors.append([col4/255, col5/255, col6/255])
                        else:
                            normals.append([col4, col5, col6])
                            
                except (ValueError, IndexError) as e:
                    self.status_display(f"Skipping malformed line: {line} ({str(e)})")
                    continue
        
        if not points:
            raise ValueError("ASC file contains no valid points")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        
        if colors and len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
        if normals and len(normals) == len(points):
            # Normalize the normals to be safe
            normals_array = np.array(normals)
            norms = np.linalg.norm(normals_array, axis=1)
            normals_array = normals_array / norms[:, np.newaxis]
            pcd.normals = o3d.utility.Vector3dVector(normals_array)
        
        return pcd

    def show_point_cloud(self):
        try:
            if self.pc_viewer.o3d_pointcloud is None:
                self.status_display.append("No point cloud to display")
                return
                
            # Show the point cloud
            self.pc_viewer.show_point_cloud()
            self.status_display.append("Displaying point cloud in Open3D viewer")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to visualize point cloud:\n{str(e)}")

    def update_status(self, message):
        self.status_display.append(message+"\n")

    def closeEvent(self, event):  # proper clean up of socket connections before closing the program
        # Handle every type of closing the window (alt+f4, red cross of the window)
        try:    
            super().closeEvent(event)
            
            # Ask for closing confirmation
            reply = QMessageBox.question(
                self, 
                'Quit',
                'Are you sure you want to quit?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                # Disconnect all devices
                self.arm_client.disconnect_all()
                event.accept()  # Let the window close
            else:
                event.ignore()  # Keep the window open

        except Exception as e:
            print(f"Error during shutdown: {e}")
            event.accept()  # Ensure window closes even if errors occur

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoboticArmGUI()
    window.show()
    sys.exit(app.exec())