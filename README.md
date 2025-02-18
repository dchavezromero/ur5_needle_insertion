# UR5 Needle Insertion - ROS 2 Workspace

This repository contains the `ur5_needle_insertion` ROS 2 workspace for UR5 robot-based needle insertion path planning. 

---
## **0. Disclosure**
The `ur5_robot_description` package was manually created by gathering the necessary resources and files from the repos below:
- [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [`urdf_files_dataset`](https://github.com/Daniella1/urdf_files_dataset/blob/main/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur5.urdf)

## **1. Prerequisites**
Ensure you have the following installed:
- **ROS 2 (Humble)** → [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Colcon** (for building ROS 2 workspaces)
- **Gazebo Classic** (for physics simulation) → [Installation Guide](https://classic.gazebosim.org/tutorials?tut=ros2_installing) **Important!!** The instructions here are for ros2 foxy, change every instance of `foxy` to `humble` when executing the commands. 
- **Git** (for cloning and managing repositories)

### **Install Required Dependencies**
```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git
```

---

## **2. Setting Up a Colcon Workspace From Scratch**
If you haven't set up a ROS 2 workspace yet, follow these steps:

### **Create a Colcon Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### **Clone This Repository**
```bash
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:<your-username>/ur5_needle_insertion.git
cd ur5_needle_insertion
```

If you **already cloned it without submodules**, initialize the submodule manually:
```bash
git submodule update --init --recursive
```

---

## **3. Install Dependencies**
Run `rosdep` to install dependencies for all packages:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## **4. Build the Workspace**
Use **colcon** to build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Source the environment:
```bash
source install/setup.bash
```
To **automatically source it in every new terminal**, add it to your `.bashrc`:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## **5. UR5 Robot Model Setup**
This repository includes the **Universal Robots ROS 2 description package** as a **submodule**:

- **Submodule Repositories:** [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)[`gazebo_ros2_control`](https://github.com/ros-controls/gazebo_ros2_control/tree/humble)
- **Paths in Workspace:** `~/ros2_ws/src/ur5_needle_insertion/Universal_Robots_ROS2_Description` & `~/ros2_ws/src/ur5_needle_insertion/gazebo_ros2_control`

The `gazebo_ros2_control` package provides us the necessary hardware plugins and controllers for the UR5 arm to use within classical gazebo. 
The `Universal_Robots_ROS2_Description` will be deleted at a later time.

---

## **6. Running a ROS 2 Node**

To launch the UR5 robot in rviz:
```bash
ros2 launch ur5_robot_description ur5_rviz.launch.py
```
To launch the UR5 robot in Gazebo Classic:
```bash
ros2 launch ur5_robot_description ur5_gazebo.launch.py
```
---

## **7. Updating the Repository**
To update your repository and pull changes from GitHub:
```bash
cd ~/ros2_ws/src/ur5_needle_insertion
git pull origin main  # Or replace 'main' with your active branch
git submodule update --recursive --remote
```

If the submodule **gazebo_ros2_control** has new changes:
```bash
cd ~/ros2_ws/src/ur5_needle_insertion/gazebo_ros2_control
git pull origin humble  # Fetch the latest updates from the upstream repo
```

---

## **8. Contributing**
If you want to contribute:
1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feature-branch
   ```
3. **Make your changes and commit**:
   ```bash
   git add .
   git commit -m "Added new feature"
   ```
4. **Push the branch and create a Pull Request**:
   ```bash
   git push origin feature-branch
   ```

---
