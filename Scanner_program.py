<<<<<<< HEAD
# -*- coding: utf-8 -*-
import gom
import socket


def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('127.0.0.1', 54321))
    server_socket.listen(5)
    # print("Server waiting for connections...")

    # Input the room temperature and exposure mode
    # gom.script.atos.set_parameters_for_new_measurements (automatic_exposure_time_mode='reference_points')
    gom.script.atos.set_parameters_for_new_measurements(automatic_exposure_time_mode='no_exposure_time_computation')
    gom.script.atos.set_acquisition_parameters(measurement_temperature=30.0)

    while True:
        try:
            client_socket, addr = server_socket.accept()
            # print(f"Connection from {addr}")

            try:
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        # print(f"Client {addr} disconnected gracefully")
                        break

                    message = data.decode('utf-8').strip()
                    # print(f"Received: {message}")

                    try:
                        # Process command
                        if message.startswith("CAPTURE"):
                            # ZEISS scan function

                            ## Scan
                            GOM_MMT_RETRIES = 10
                            GOM_MMT_ERROR_CODES = ['MPROJ-0037']
                            while True:
                                try:
                                    gom.script.atos.insert_scan_measurement(min_measurement_exposure_time=0.055542,
                                                                            reference_point_exposure_time=0.02114)
                                except RuntimeError as ex:
                                    if GOM_MMT_ERROR_CODES and not ex.args[0] in GOM_MMT_ERROR_CODES: raise
                                    GOM_MMT_RETRIES -= 1
                                    if GOM_MMT_RETRIES <= 0: raise
                                else:
                                    break

                            response = "SCAN_COMPLETED"

                        elif message.startswith("SAVE"):
                            # Expect format: "SAVE:/path/to/file"
                            save_path = message.split(":")[1] + ":" + message.split(":")[
                                2]  # get the C: part + the path in itself

                            # Save the point cloud with ZEISS software

                            gom.script.sys.restore_point_selection(
                                elements=[gom.app.project.parts['部件'].actual],
                                stage=gom.app.project.stages['阶段 1'])

                            gom.script.sys.edit_creation_parameters(
                                element=gom.app.project.parts['部件'].actual,
                                lock_after_finalize_draft=False,
                                name='Data',
                                polygonization_process='standard',
                                recalculation_behavior='recalc_without_report_pages')

                            gom.script.cad.hide_element(elements=[gom.app.project.measurement_series['Scan 1']])

                            gom.script.cad.show_element(elements=[gom.app.project.parts['部件'].actual])
                            # C:/Users/Zeiss/Desktop/Nathan/pcl.ply
                            gom.script.sys.export_ply(
                                binary=False,
                                color=False,
                                deviation=False,
                                elements=gom.ElementSelection(gom.app.project.parts['部件'].actual, {
                                    'category': ['key', 'elements', 'part', gom.app.project.parts['部件'],
                                                 'explorer_category', 'actual_part']}),
                                export_stages_mode='current',
                                file=save_path,
                                length_unit='default')

                            response = f"SAVED_TO:{save_path}"

                        elif message == "DISCONNECT":
                            client_socket.sendall(b"Disconnected")
                            client_socket.close()
                            server_socket.close()

                        else:
                            response = "ERROR:UNKNOWN_COMMAND"

                    except Exception as e:
                        response = f"SCAN ERROR:{str(e)}"
                    # print(f"Command failed: {response}")

                    finally:
                        client_socket.sendall(response.encode('utf-8'))

            except socket.timeout:
                # print(f"Client {addr} timeout")
                break
            except ConnectionResetError:
                # print(f"Client {addr} disconnected abruptly")
                break
            except Exception as e:
                # print(f"Unexpected error with {addr}: {str(e)}")
                break

        except socket.timeout:
            continue  # No new connections, retry accept()
        except KeyboardInterrupt:
            # print("\nServer shutting down...")
            break
        except Exception as e:
            # print(f"Critical server error: {str(e)}")
            break
        finally:
            client_socket.close()


if __name__ == "__main__":
    start_server()

=======
# -*- coding: utf-8 -*-
import gom
import socket


def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('127.0.0.1', 54321))
    server_socket.listen(5)
    # print("Server waiting for connections...")

    # Input the room temperature and exposure mode
    # gom.script.atos.set_parameters_for_new_measurements (automatic_exposure_time_mode='reference_points')
    gom.script.atos.set_parameters_for_new_measurements(automatic_exposure_time_mode='no_exposure_time_computation')
    gom.script.atos.set_acquisition_parameters(measurement_temperature=30.0)

    while True:
        try:
            client_socket, addr = server_socket.accept()
            # print(f"Connection from {addr}")

            try:
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        # print(f"Client {addr} disconnected gracefully")
                        break

                    message = data.decode('utf-8').strip()
                    # print(f"Received: {message}")

                    try:
                        # Process command
                        if message.startswith("CAPTURE"):
                            # ZEISS scan function

                            ## Scan
                            GOM_MMT_RETRIES = 10
                            GOM_MMT_ERROR_CODES = ['MPROJ-0037']
                            while True:
                                try:
                                    gom.script.atos.insert_scan_measurement(min_measurement_exposure_time=0.055542,
                                                                            reference_point_exposure_time=0.02114)
                                except RuntimeError as ex:
                                    if GOM_MMT_ERROR_CODES and not ex.args[0] in GOM_MMT_ERROR_CODES: raise
                                    GOM_MMT_RETRIES -= 1
                                    if GOM_MMT_RETRIES <= 0: raise
                                else:
                                    break

                            response = "SCAN_COMPLETED"

                        elif message.startswith("SAVE"):
                            # Expect format: "SAVE:/path/to/file"
                            save_path = message.split(":")[1] + ":" + message.split(":")[
                                2]  # get the C: part + the path in itself

                            # Save the point cloud with ZEISS software

                            gom.script.sys.restore_point_selection(
                                elements=[gom.app.project.parts['部件'].actual],
                                stage=gom.app.project.stages['阶段 1'])

                            gom.script.sys.edit_creation_parameters(
                                element=gom.app.project.parts['部件'].actual,
                                lock_after_finalize_draft=False,
                                name='Data',
                                polygonization_process='standard',
                                recalculation_behavior='recalc_without_report_pages')

                            gom.script.cad.hide_element(elements=[gom.app.project.measurement_series['Scan 1']])

                            gom.script.cad.show_element(elements=[gom.app.project.parts['部件'].actual])
                            # C:/Users/Zeiss/Desktop/Nathan/pcl.ply
                            gom.script.sys.export_ply(
                                binary=False,
                                color=False,
                                deviation=False,
                                elements=gom.ElementSelection(gom.app.project.parts['部件'].actual, {
                                    'category': ['key', 'elements', 'part', gom.app.project.parts['部件'],
                                                 'explorer_category', 'actual_part']}),
                                export_stages_mode='current',
                                file=save_path,
                                length_unit='default')

                            response = f"SAVED_TO:{save_path}"

                        elif message == "DISCONNECT":
                            client_socket.sendall(b"Disconnected")
                            client_socket.close()
                            server_socket.close()

                        else:
                            response = "ERROR:UNKNOWN_COMMAND"

                    except Exception as e:
                        response = f"SCAN ERROR:{str(e)}"
                    # print(f"Command failed: {response}")

                    finally:
                        client_socket.sendall(response.encode('utf-8'))

            except socket.timeout:
                # print(f"Client {addr} timeout")
                break
            except ConnectionResetError:
                # print(f"Client {addr} disconnected abruptly")
                break
            except Exception as e:
                # print(f"Unexpected error with {addr}: {str(e)}")
                break

        except socket.timeout:
            continue  # No new connections, retry accept()
        except KeyboardInterrupt:
            # print("\nServer shutting down...")
            break
        except Exception as e:
            # print(f"Critical server error: {str(e)}")
            break
        finally:
            client_socket.close()


if __name__ == "__main__":
    start_server()

>>>>>>> 1f5935a291bc6e80918cf3a2c91adbc8e9a8f452
