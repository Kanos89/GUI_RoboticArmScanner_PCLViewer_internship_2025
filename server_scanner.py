import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('127.0.0.1', 54321))
    server_socket.listen(5)
    server_socket.settimeout(1.0)  # Prevent accept() from hanging indefinitely
    print("Server waiting for connections...")
    
    while True:
        try:
            client_socket, addr = server_socket.accept()
            client_socket.settimeout(10.0)  # Per-command timeout
            print(f"Connection from {addr}")
            
            try:
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        print(f"Client {addr} disconnected gracefully")
                        break

                    message = data.decode('utf-8').strip()
                    print(f"Received: {message}")

                    try:
                        # Process command
                        if message.startswith("CAPTURE"):
                            # Insert scan function here
                            response = "SCAN_COMPLETE"
                            
                        elif message.startswith("SAVE"):
                            # Expect format: "SAVE:/path/to/file"
                            save_path = message.split(":")[1]
                            # Insert save function here
                            response = f"SAVED_TO:{save_path}"
                            
                        elif message == "DISCONNECT":
                            client_socket.sendall(b"GOODBYE")
                            break
                            
                        else:
                            response = "ERROR:UNKNOWN_COMMAND"
                            
                    except Exception as e:
                        response = f"ERROR:{str(e)}"
                        print(f"Command failed: {response}")
                        
                    finally:
                        client_socket.sendall(response.encode('utf-8'))

            except socket.timeout:
                print(f"Client {addr} timeout")
            except ConnectionResetError:
                print(f"Client {addr} disconnected abruptly")
            except Exception as e:
                print(f"Unexpected error with {addr}: {str(e)}")
            finally:
                client_socket.close()
                print(f"Connection with {addr} closed")

        except socket.timeout:
            continue  # No new connections, retry accept()
        except KeyboardInterrupt:
            print("\nServer shutting down...")
            break
        except Exception as e:
            print(f"Critical server error: {str(e)}")

if __name__ == "__main__":
    start_server()