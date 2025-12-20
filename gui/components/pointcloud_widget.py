import numpy as np
import open3d as o3d
from PyQt6.QtWidgets import QWidget,QApplication,QLabel,QVBoxLayout
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt

class PointCloudWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.o3d_geometry = None
        
        self.placeholder_label = QLabel("No point cloud loaded\n\n"
                                      "Load a point cloud file to visualize\n"
                                      "or capture a new scan")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setWordWrap(True)
        self.placeholder_label.setStyleSheet("color: #6c757d; font-style: italic;")
        self.placeholder_label.setFont(QFont("Arial", 10))
        
        layout = QVBoxLayout()
        layout.addWidget(self.placeholder_label)
        self.setLayout(layout)

    def update_point_cloud(self, data):
        if isinstance(data, o3d.geometry.PointCloud):
            self.o3d_geometry = data
            point_count = len(data.points)
            self.placeholder_label.setText(f"Point cloud ready\n\n"
                                        f"Points: {point_count:,}\n"
                                        f"Click 'View' to visualize")
            
        elif isinstance(data, o3d.geometry.TriangleMesh):
            self.o3d_geometry = data
            vertex_count = len(data.vertices)
            triangle_count = len(data.triangles)
            self.placeholder_label.setText(f"Mesh ready\n\n"
                                        f"Vertices: {vertex_count:,}\n"
                                        f"Triangles: {triangle_count:,}\n"
                                        f"Click 'View' to visualize")
            
        elif isinstance(data, np.ndarray):
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(data)
            self.o3d_geometry = pc
            point_count = len(data)
            self.placeholder_label.setText(f"Point cloud ready\n\n"
                                        f"Points: {point_count:,}\n"
                                        f"Click 'View' to visualize")
        else:
            raise TypeError("Unsupported data type for PointCloudWidget")
        
        self.setToolTip(f"Geometry loaded")


    def show_point_cloud(self, show_normals=False, title_suffix=""):
        """Visualize the geometry using Open3D, centered on screen"""
        if self.o3d_geometry is not None:
            # Get the screen geometry to center the window
            screen = QApplication.primaryScreen()
            screen_geometry = screen.geometry()
            screen_width = screen_geometry.width()
            screen_height = screen_geometry.height()
            
            # Set window size (usual 1080p)
            window_width = 1920
            window_height = 1080
            
            # Calculate center position
            x_pos = (screen_width - window_width) // 2
            y_pos = ((screen_height - window_height) // 2) + 30
            
            # Manage PointCloud vs TriangleMesh
            if isinstance(self.o3d_geometry, o3d.geometry.PointCloud):
                point_count = len(self.o3d_geometry.points)
                geometry_type = "Point Cloud"
            elif isinstance(self.o3d_geometry, o3d.geometry.TriangleMesh):
                point_count = len(self.o3d_geometry.vertices)  # vertices, not points!
                geometry_type = "Mesh"
            else:
                point_count = 0
                geometry_type = "Geometry"
            
            # Create window title
            window_title = f"{geometry_type} Viewer - {point_count:,} elements"
            if title_suffix:
                window_title += f" - {title_suffix}"
            
            vis = o3d.visualization.Visualizer()
            vis.create_window(
                window_name=window_title,
                width=window_width,
                height=window_height,
                left=x_pos,
                top=y_pos
            )
            vis.add_geometry(self.o3d_geometry)

            # Render settings - seulement pour PointCloud
            opt = vis.get_render_option()
            opt.background_color = np.array([0.6, 0.6, 0.6])
            
            # Apply settings only if a PointCloud
            if isinstance(self.o3d_geometry, o3d.geometry.PointCloud):
                opt.point_size = 2.0
                opt.point_show_normal = show_normals

            vis.run()
            vis.destroy_window()

    
    def clear_point_cloud(self):
        """Clear the current point cloud and reset the display"""
        self.o3d_geometry = None
        self.placeholder_label.setText("No point cloud loaded\n\n"
                                        "Load a point cloud file to visualize\n"
                                        "or capture a new scan")
        self.placeholder_label.setStyleSheet("color: #6c757d; font-style: italic;")
        self.setToolTip("No point cloud loaded")