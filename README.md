# GUI Robotic Arm Scanner & PCL Viewer (Internship HUST 2025)

### Built with : #Python #PyQt6  #TCP/IP Sockets #Open3D #ABB RobotStudio (RAPID) #ZEISS inspect (Python)

This repository features a custom-built supervisory control application designed to automate industrial 3D scanning workflows by seamlessly coordinating an industrial robotic arm with a 3D scanner.

### The Challenge
The core obstacle was the 3D scanner operated exclusively within the manufacturer's proprietary software suite (ZEISS), while the robotic arm required its own dedicated control environment (ABB RobotStudio). Because both systems were closed off from one another, a centralized, automated control flow was impossible out of the box.

### My Solution
To bridge this gap, I developed a centralized Python-based supervisory application. Architected as a **Dual-Client System**, the application initiates and manages simultaneous TCP/IP socket connections to both software environments, which act as dedicated servers waiting for incoming requests. 

The software orchestrates the automation loop through four core tasks:
1. **Synchronized TCP/IP Communication:** Establishes and maintains concurrent client-server sockets with the robot controller and the 3D scanner.
2. **Automated Motion Dispatching:** Sends positions corresponding to saved coordinates to the robot server, sequencing the arm through predefined inspection viewpoints around the target component.
3. **Remote Triggering:** Signals the scanner server to capture a high-resolution 3D scan the moment the robot confirms it has reached a stable inspection node.
4. **Data Aggregation & Visualization:** Pulls and processes the resulting point cloud data files (`.ply`/`.pcd`) for real-time localized 3D rendering.

This software would effectively bridge the gap between the two isolated systems, enabling
a seamless and automated digitization workflow

## Quick showcase of the project 
### Showcase of the automated scan :
https://github.com/user-attachments/assets/07767f72-772d-420c-b0b9-bdae92c39b7c

### The main user interface: 
<img width="755" height="677" alt="image" src="https://github.com/user-attachments/assets/7839975c-17e7-4151-aa51-e0f48739fdf4" />

### Robotstudio simulation integration :
<img width="1306" height="679" alt="image" src="https://github.com/user-attachments/assets/4f994e48-4141-4edc-9d35-4de18151a633" />

### The modular project structure : 
<img width="784" height="494" alt="image" src="https://github.com/user-attachments/assets/856167d2-7f47-43b4-b5fb-87d238c573c1" />

## Built With
* **Language:** Python 3
* **GUI Framework:** PyQt6
* **3D Graphics Engine:** Open3D (Point Cloud processing and visualization)
* **Industrial Simulation:** ABB RobotStudio & RAPID programming
* **Networking Protocol:** TCP/IP Sockets (Dual-Client Architecture)
  

## Methodology

The project followed an iterative, exploratory development lifecycle, prioritizing rapid prototyping alongside systematic validation:

1. **Preliminary Research & Framework Mastery:** Conducted comprehensive documentation deep-dives into `PyQt6` for responsive multi-threaded UI design and `Open3D` for low-overhead point cloud manipulation.
2. **Collaborative & Assisted Problem-Solving:** Utilized peer-to-peer technical reviews within the laboratory to approach complex algorithmic bottlenecks from multiple engineering perspectives. Incorporated AI coding assistants to accelerate syntax debugging and conceptual discovery, critically auditing and refactoring suggested code blocks to align with the overarching multi-file system architecture.
3. **Rigorous Testing Cycle:**
   * **Isolated Prototyping:** Core functionalities (socket handshakes, string parsing, data rendering) were rapidly verified in single-script sandboxes.
   * **Virtual Simulation:** Motion trajectories and communication handshakes were rigorously validated inside ABB RobotStudio to ensure collision-free kinematics and logic stability.
   * **Hardware-in-the-Loop Validation:** Deployed the verified codebase to physical laboratory hardware, dynamically adjusting for real-world environmental factors such as active tool center point (TCP) end-effector offsets.
   * **Architectural Refactoring:** As system complexity grew, successfully refactored a massive monolithic script into a clean, decoupled, modular package structure to maximize maintainability.


This approach, while highly autonomous and offering great freedom, required strong
personal commitment and consistent problem-solving to navigate the absence of strict
external deadlines and define the project's direction independently.


## Future Outlook / Next Steps
* **Scan Sequence Stability:** Investigate and patch thread-blocking conditions during extended automated data collection sequences to ensure continuous execution.
* **Surface Normal Correction:** Refine the normal estimation algorithms in Open3D to systematically handle and correct vector orientation inversions on complex or reflective geometries.
