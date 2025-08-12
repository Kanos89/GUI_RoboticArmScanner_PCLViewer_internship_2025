import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 54321))
    server_socket.listen(5)
    print("Server waiting for connections...")
    
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")
        
        try:
            while True:  # Persistent connection loop
                data = client_socket.recv(1024)
                if not data:
                    break  # Client disconnected
                    
                message = data.decode('utf-8')
                print(f"Received: {message}")
                
                # Process command
                if message.startswith("CAPTURE"):
                    response = f"SCAN_COMPLETE"
                else:
                    response = f"SCAN_INCOMPLETE"
                
                client_socket.sendall(response.encode('utf-8'))
                
        except ConnectionResetError:
            print("Client disconnected abruptly")
        except Exception as e:
            print(f"Error: {str(e)}")
        finally:
            client_socket.close()
            print("Connection closed")


if __name__ == "__main__":
    start_server()