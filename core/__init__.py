"""
Core package for robotic arm and scanner control.
Contains the main controller and communication protocols.
"""

from core.station_controller import StationController, AutoScanWorker
from core.protocols import DeviceType, ArmCommands, ScannerCommands, ConnectionManager

__version__ = "1.0.0"
__author__ = "Kanos89"
__all__ = [
    'StationController',
    'AutoScanWorker',
    'DeviceType', 
    'ArmCommands',
    'ScannerCommands',
    'ConnectionManager'
]