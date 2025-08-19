from PyQt6.QtCore import QThread, pyqtSignal
import numpy as np
from core.protocols import DeviceType, ArmCommands, ScannerCommands, ConnectionManager

class StationController(QThread):
    arm_status = pyqtSignal(str)
    scanner_status = pyqtSignal(str)
    scan_complete = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.arm_socket = None
        self.scanner_socket = None
        self.arm_connected = False
        self.scanner_connected = False

    def connect_device(self, device_type: 'DeviceType', host: str, port: int) -> bool:
        try:
            sock = ConnectionManager.create_connection(host, port)
            if device_type == DeviceType.ARM:
                self.arm_socket = sock
                self.arm_connected = True
                self.arm_status.emit(f"Connected to arm at {host}:{port}")
            else:
                self.scanner_socket = sock
                self.scanner_connected = True
                self.scanner_status.emit(f"Connected to scanner at {host}:{port}")
            return True
        except ConnectionError as e:
            if device_type == DeviceType.ARM:
                self.arm_status.emit(str(e))
            else:
                self.scanner_status.emit(str(e))
            return False

    def send_command(self, device_type: 'DeviceType', command: str):
        sock = self.arm_socket if device_type == DeviceType.ARM else self.scanner_socket
        status_signal = self.arm_status if device_type == DeviceType.ARM else self.scanner_status
        
        if not (sock and (self.arm_connected if device_type == DeviceType.ARM else self.scanner_connected)):
            status_signal.emit(f"Not connected to {device_type.name.lower()}")
            return None

        success, response = ConnectionManager.send_command(sock, command)
        status_signal.emit(f"{device_type.name} response: {response}")
        return response if success else None

    def move_to_position(self, pos_name: str):
        return self.send_command(
            DeviceType.ARM,
            ArmCommands.MOVE.format(position=pos_name)
        )

    def capture_scan(self) -> bool:
        response = self.send_command(DeviceType.SCANNER, ScannerCommands.CAPTURE)
        return response == "SCAN_COMPLETED"

    def disconnect_device(self, device_type: 'DeviceType') -> None:
        sock = self.arm_socket if device_type == DeviceType.ARM else self.scanner_socket
        status_signal = self.arm_status if device_type == DeviceType.ARM else self.scanner_status
        disconnect_cmd = ArmCommands.DISCONNECT if device_type == DeviceType.ARM else ScannerCommands.DISCONNECT

        if sock:
            response = ConnectionManager.safe_disconnect(sock, disconnect_cmd)
            status_signal.emit(f"{device_type.name} disconnect: {response}")
            
            if device_type == DeviceType.ARM:
                self.arm_connected = False
                self.arm_socket = None
            else:
                self.scanner_connected = False
                self.scanner_socket = None

    def disconnect_all(self) -> None:
        self.disconnect_device(DeviceType.ARM)
        self.disconnect_device(DeviceType.SCANNER)

