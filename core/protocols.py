from enum import Enum, auto
from dataclasses import dataclass
import socket
from typing import Optional, Tuple

class DeviceType(Enum):
    ARM = auto()
    SCANNER = auto()

@dataclass
class Command:
    """Base command structure for all devices"""
    device: DeviceType
    command: str
    timeout: float = 20.0

class ArmCommands:
    MOVE = "MOVE {position}"
    DISCONNECT = "DISCONNECT"

class ScannerCommands:
    CAPTURE = "CAPTURE"
    DISCONNECT = "DISCONNECT"
    SAVE_POINTCLOUD = "SAVE:{file_path}"

class ConnectionManager:
    @staticmethod
    def create_connection(host: str, port: int, timeout: float = 5.0) -> Optional[socket.socket]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((host, port))
            return sock
        except (socket.error, socket.timeout) as e:
            raise ConnectionError(f"Connection failed: {str(e)}")

    @staticmethod
    def send_command(sock: socket.socket, command: str, timeout: float = 20.0) -> Tuple[bool, str]:
        try:
            sock.settimeout(timeout)
            sock.sendall(command.encode('utf-8'))
            response = sock.recv(1024).decode('utf-8').strip()
            return True, response
        except socket.timeout:
            return False, "Timeout waiting for response"
        except socket.error as e:
            return False, f"Socket error: {str(e)}"

    @staticmethod
    def safe_disconnect(sock: socket.socket, disconnect_cmd: str, timeout: float = 3.0) -> str:
        try:
            if sock:
                sock.sendall(disconnect_cmd.encode('utf-8'))
                sock.settimeout(timeout)
                response = sock.recv(1024).decode('utf-8')
                sock.close()
                return response
        except Exception:
            pass
        return "Disconnected (no response)"