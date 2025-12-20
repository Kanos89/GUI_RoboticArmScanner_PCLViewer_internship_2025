# gui/main_window.py

import os
import numpy as np 
import open3d as o3d

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, 
    QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QTextEdit, QMessageBox, QFileDialog,
    QComboBox, QDialog
)
from PyQt6.QtCore import QThread
from PyQt6.QtGui import QFont

from core import StationController, DeviceType, ScannerCommands, AutoScanWorker
from gui.components import PointCloudWidget, SettingsDialog
from processing import PointCloudManager
from utils.constants import (
    DEFAULT_ARM_HOST, DEFAULT_ARM_PORT,
    DEFAULT_SCANNER_HOST, DEFAULT_SCANNER_PORT,
    SCAN_POSITIONS, POINTCLOUD_EXTENSIONS
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm & Scanner Control System")
        self.setGeometry(100, 100, 700, 600)
        self.arm_client = StationController()  
        self.scans = []
        self.init_ui()
        self.setup_connections()
        self.pointcloud_manager = PointCloudManager()

        # Auto scan thread
        self.auto_scan_thread = None
        self.auto_scan_worker = None

    def init_ui(self):
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left Panel (Connection and Control)
        left_panel = QVBoxLayout()

        # Arm Connection Group
        arm_conn_group = QGroupBox("Arm Connection")
        arm_conn_layout = QVBoxLayout()
        
        self.arm_host_input = QLineEdit(DEFAULT_ARM_HOST)
        self.arm_port_input = QLineEdit(DEFAULT_ARM_PORT)
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
        
        self.scanner_host_input = QLineEdit(DEFAULT_SCANNER_HOST)
        self.scanner_port_input = QLineEdit(DEFAULT_SCANNER_PORT)
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
        self.pos_combo.addItems(SCAN_POSITIONS)
        self.move_btn = QPushButton("Move to Position")
        self.move_btn.setEnabled(False)
        
        self.capture_btn = QPushButton("Capture Scan")
        self.capture_btn.setEnabled(False)

        # --- Automatic Scan Section ---
        control_layout.addWidget(QLabel("Select Position:"))
        control_layout.addWidget(self.pos_combo)
        control_layout.addWidget(self.move_btn)
        control_layout.addWidget(self.capture_btn)
        control_layout.addWidget(QLabel("Automatic Scan:"))
        self.auto_scan_btn = QPushButton("Start Automatic Scan")
        self.auto_scan_btn.setEnabled(False)
        control_layout.addWidget(self.auto_scan_btn)

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

        self.save_btn = QPushButton("Save Point Cloud")
        self.save_btn.setEnabled(False)

        self.open_pcl_btn = QPushButton("Open file")

        self.view_btn = QPushButton("View Point Cloud")
        self.view_btn.setEnabled(False)

        pc_layout.addWidget(self.save_btn)
        pc_layout.addWidget(QLabel("PCL to view:"))
        pc_layout.addWidget(self.open_pcl_btn)
        pc_layout.addWidget(self.view_btn)

        separator_label = QLabel("────────────────────────")
        separator_label.setStyleSheet("color: #ccc;")
        pc_layout.addWidget(separator_label)

        self.pc_viewer = PointCloudWidget()
        pc_layout.addWidget(self.pc_viewer)

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
        self.auto_scan_btn.clicked.connect(self.start_auto_scan)
        
        # Point Cloud
        self.save_btn.clicked.connect(self.save_point_cloud)
        self.view_btn.clicked.connect(self.show_point_cloud)
        self.open_pcl_btn.clicked.connect(self.open_pcl_dialog)
        
        # Status
        self.arm_client.arm_status.connect(self.update_status)
        self.arm_client.scanner_status.connect(self.update_status)

    def start_auto_scan(self):
        self.update_status("Starting automatic scan...")
        self.auto_scan_btn.setEnabled(False)

        self.auto_scan_thread = QThread()
        self.auto_scan_worker = AutoScanWorker(self.arm_client)
        self.auto_scan_worker.moveToThread(self.auto_scan_thread)

        self.auto_scan_thread.started.connect(self.auto_scan_worker.run)
        self.auto_scan_worker.finished.connect(self.auto_scan_thread.quit)
        self.auto_scan_worker.finished.connect(self.auto_scan_worker.deleteLater)
        self.auto_scan_thread.finished.connect(self.auto_scan_thread.deleteLater)

        self.auto_scan_worker.progress.connect(self.update_status)
        self.auto_scan_worker.finished.connect(
            lambda: self.auto_scan_btn.setEnabled(True)
        )

        self.auto_scan_thread.start()

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

        if self.arm_client.connect_device(DeviceType.ARM, host, port):
            self.arm_connect_btn.setEnabled(False)
            self.arm_disconnect_btn.setEnabled(True)
            self.move_btn.setEnabled(True)
            self.auto_scan_btn.setEnabled(True)

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

        if self.arm_client.connect_device(DeviceType.SCANNER, host, port):
            self.scanner_connect_btn.setEnabled(False)
            self.scanner_disconnect_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)
            self.save_btn.setEnabled(True)
            
    def disconnect_arm(self):
        self.arm_client.disconnect_device(DeviceType.ARM)
        self.arm_connect_btn.setEnabled(True)
        self.arm_disconnect_btn.setEnabled(False)
        self.move_btn.setEnabled(False)
        self.auto_scan_btn.setEnabled(False)

    def disconnect_scanner(self):
        self.arm_client.disconnect_device(DeviceType.SCANNER)
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
                "pcl.ply",
                "All Files (*)"
            )

            if not file_path:
                return

            file_path = os.path.splitext(file_path)[0] + '.ply'
            self.update_status(f"Attempting to save point cloud to {file_path}...")

            # Use the new command format
            response = self.arm_client.send_command(
                DeviceType.SCANNER,
                ScannerCommands.SAVE_POINTCLOUD.format(file_path=file_path)
            )

            if response:
                self.view_btn.setEnabled(True)
            else:
                self.status_display.append("Failed to receive confirmation from scanner")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save point cloud:\n{str(e)}")
            self.status_display.append(f"Save error: {str(e)}")
        finally:
            self.view_btn.setEnabled(True)
                
    def open_pcl_dialog(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select a File",
            "",
            f"Point Cloud Files ({POINTCLOUD_EXTENSIONS});; All Files (*)"
        )
        
        if file_path:
            try:
                # load the file
                self.point_cloud = self.pointcloud_manager.read_pointcloud_file(file_path)
                
                # update the widget
                self.pc_viewer.update_point_cloud(self.point_cloud)
                
                # placeholder_label will be updated automatically
                self.view_btn.setEnabled(True)
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file:\n{str(e)}")
                self.point_cloud = None
                # Clear the widget if error triggered
                self.pc_viewer.clear_point_cloud()


    def show_point_cloud(self):
        try:
            if self.point_cloud is None:
                self.status_display.append("No point cloud to display")
                return

            # Open settings dialog
            dialog = SettingsDialog(self)
            if dialog.exec() == QDialog.DialogCode.Accepted:
                settings = dialog.get_settings()
                pc = self.point_cloud

                if settings["downsample"]:
                    voxel_size = 0.9  
                    pc = self.pointcloud_manager.downsample(pc, voxel_size=voxel_size)
                    self.update_status("Downsampling applied")

                if settings["estimate_normals"]:
                    # normals are estimated on the downsampled pcl
                    pc = self.pointcloud_manager.estimate_normals(pc)
                    self.update_status("Normals estimated on downsampled cloud")


                title_suffix = ""
                if settings["downsample"]:
                    title_suffix += "Downsampled"

                if settings["estimate_normals"]:
                    if title_suffix:
                        title_suffix += " + "
                    title_suffix += "With Normals"

                if settings["meshing"]:
                    mesh = self.pointcloud_manager.poisson_meshing(pc)
                    self.pc_viewer.update_point_cloud(mesh)
                    self.update_status("Mesh generated")
                    title_suffix="Meshed"
                    self.pc_viewer.show_point_cloud(show_normals=False, title_suffix=title_suffix) # no normal visualisation on meshed pcl
                else:
                    self.pc_viewer.update_point_cloud(pc)
                    self.update_status("Point cloud ready for visualization")
                    self.pc_viewer.show_point_cloud(
                        show_normals=settings["estimate_normals"],
                        title_suffix=title_suffix
                    )

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