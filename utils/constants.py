"""
Application constants and default values
"""

# Connection defaults
DEFAULT_ARM_HOST = "192.168.125.1"
DEFAULT_ARM_PORT = "1234"
DEFAULT_SCANNER_HOST = "127.0.0.1"
DEFAULT_SCANNER_PORT = "54321"

# Timeouts
CONNECTION_TIMEOUT = 1.0
COMMAND_TIMEOUT = 60.0 # Scanner needs to save PCL, take much time
DISCONNECT_TIMEOUT = 3.0

# Scan positions (for automated scanning)
SCAN_POSITIONS = [
    "Scan Position 1", 
    "Scan Position 2",
    "Scan Position 3",
    "Scan Position 4"
]

# File extensions
POINTCLOUD_EXTENSIONS = "*.xyz *.xyzn *.xyzrgb *.pts *.ply *.pcd *.asc"