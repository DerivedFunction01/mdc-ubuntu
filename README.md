# Miniature Driverless Car

This repository integrates the **Azure Kinect camera** with **ROS 2** using the [`Azure_Kinect_ROS_Driver`](https://github.com/microsoft/Azure_Kinect_ROS_Driver) and connects it with two control packages:

- **Azure_Kinect_ROS_Driver** → Starts the Azure Kinect camera node in ROS 2  
- **laptop** → Interfaces with an Xbox controller or keyboard
- **mdc_car** → Runs and controls the vehicle  

Together, these packages allow sensor input from the Kinect and user input from the controller to drive the car through ROS 2.

---

## 📂 Repository Structure

- `Azure_Kinect_ROS_Driver/` → ROS 2 driver for the Azure Kinect  
- `laptop/` → ROS 2 node for Xbox controller input  
- `mdc_car/` → ROS 2 node that controls the car  
- `src/` → Source code for `laptop` and `mdc_car` packages  
- `scripts/*.sh` → Convenience scripts for building and running the system  
  - `launch.sh` → Launch the nav2 + rtabmap stack.
  - `initAzure.sh` → Setup for Azure Kinect driver  
  - `initMDC.sh` → Setup for car controller  
  - `launch-old.sh` → Launches Azure Kinect with rtabmap or nav2 + rtabmap
  - `runcar.sh` → Runs the full car control system  
  - `xbox.sh` → Runs the car xbox controller or keyboard
  - `send-db.sh` → Send rtabmap data to the car
  - `deploy.sh` → Send the git repo when connected via ssh. NOTE: move the db to a different location with `move-db.sh`
  - `move-db.sh` → Move the `db` with `m` or restore with `r`

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
git clone <repo-url>
cd <repo-name>
````

### 2. Build the workspace

Instead of running `colcon build` manually, use the provided scripts:

```bash
./scripts/initAzure.sh   # Build and setup Azure Kinect driver
./scripts/initMDC.sh     # Build and setup car controller
```

### 3. Run the system

Start the full pipeline:

```bash
./scripts/runcar.sh # In the car
./scripts/xbox.sh r k # On the laptop run keyboard (or c for laptop)
./scripts/launch.sh # See the file for full options (No args run full stack)
```

This will:

1. Launch the Azure Kinect driver
2. Start the Xbox controller node (`laptop`)
3. Run the car node (`mdc_car`)

---

## 🛠 Development Notes

* The repository is configured for **ROS 2 Humble**.
* A `devcontainer` configuration is included for development.

  * This makes it easier to get started on a fresh machine, but it is not required at runtime.
* Scripts in the root directory are the main entry points for building and running.

### X11 Forwarding
```sh
# On server side (ORIN NX):
# Make sure xauth is installed
# Open a text editor with sudo privileges to edit /etc/ssh/sshd_config
sudo vim /etc/ssh/sshd_config
```
* Uncomment `ForwardX11`
* Uncomment `X11DisplayOffset 10`
* Uncomment `X11Forwarding yes`
* Uncomment `Port 22`
```sh
# Run this command to restart
sudo systemctl restart sshd

# On client side:
Add ForwardX11 yes to ~/.ssh/config

# Run this to use X11 Forwarding:
# ssh -X capstone@mdc-nx.local
ssh -X nx
```
---
### Expired ROS2 GPG Key
```sh
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Update the package list again
sudo apt update
```

### Using a visual monitor
If you are using a monitor connected to the Jetson, you may need to set the `xrandr -d :0` to get the correct display.

```sh
## System Diagram
![](./Chart/System%20Flow.svg)

## 📌 Requirements

* ROS 2 Humble
* [Azure Kinect SDK](https://learn.microsoft.com/en-us/azure/kinect-dk/) installed on host machine
* Xbox controller (wired or wireless via dongle)
* Configuration and Setup in the `devcontainer`
---

## WSL 2
In Powershell with administrative privileges, you must copy and paste `attach-devices.ps1` to allow Docker/WSL2 to access USB connected devices. You may need to add a `.wslconfig` in  `C:\Users\_name_` increase allowed usage (https://learn.microsoft.com/en-us/windows/wsl/wsl-config).
