"""
Point cloud processing utilities - ASC file handling, filtering, and processing
"""
import numpy as np
import open3d as o3d
from typing import Optional, Tuple, List
import os

class PointCloudManager:
    """
    Comprehensive point cloud management system.
    Handles file I/O, processing, filtering, and visualization settings.
    """
    
    def __init__(self):
        self.current_pointcloud = None
        self.processed_pointcloud = None
        self.filters = {
            'voxel_size': 0.01,
            'remove_outliers': True,
            'outlier_neighbors': 20,
            'outlier_std_ratio': 2.0
        }
    
    @staticmethod
    def read_asc_file(file_path: str) -> o3d.geometry.PointCloud:
        """Optimized ASC file reader using numpy"""
        try:
            # Read entire file at once
            data = np.loadtxt(file_path, comments='#')
            
            # Extract components
            points = data[:, :3]
            colors = data[:, 3:6]/255 if data.shape[1] >=6 else None
            normals = data[:, 3:6] if (data.shape[1] >=6 and 
                                    np.all(np.isclose(np.linalg.norm(data[:,3:6], axis=1), 1))) else None

            # Create point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)
            if normals is not None:
                pcd.normals = o3d.utility.Vector3dVector(normals)
                
            return pcd
            
        except Exception as e:
            raise ValueError(f"ASC read error: {str(e)}")
    
    @staticmethod
    def read_pointcloud_file(file_path: str) -> o3d.geometry.PointCloud:
        """Read various point cloud file formats"""
        if file_path.lower().endswith('.asc'):
            return PointCloudManager.read_asc_file(file_path)
        else:
            return o3d.io.read_point_cloud(file_path)
        
    def downsample(self, pc, voxel_size=0.01):
        if len(pc.points) < 1000:  # Don't downsample if already small
            return pc
        else :
            downsampled = pc.voxel_down_sample(voxel_size)     
        return downsampled

    def estimate_normals(self, pc, radius=0.01, max_nn=30):
        """Estimate normals for the point cloud."""
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
        )
        return pc

    def poisson_meshing(self, pc, depth=8):
        """Create mesh from point cloud using Poisson reconstruction."""
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pc, depth=depth
        )
        # Crop mesh to bounding box of the original point cloud
        bbox = pc.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)
        return mesh
    
    def get_pointcloud_info(self, pcd: o3d.geometry.PointCloud) -> List[str]:
        """Get formatted information about point cloud"""
        info = []
        info.append(f"Points: {len(pcd.points)}")
        if pcd.has_colors():
            info.append("Contains color information")
        if pcd.has_normals():
            info.append("Contains normal vectors")
        if pcd.has_covariances():
            info.append("Contains covariance matrices")
        return info
    
    @staticmethod
    def validate_pointcloud(pcd: o3d.geometry.PointCloud) -> bool:
        """Validate that point cloud contains data"""
        return pcd is not None and pcd.has_points()
