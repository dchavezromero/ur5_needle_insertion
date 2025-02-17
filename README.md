# UR5 Needle Insertion - ROS 2 Workspace

This repository contains the `ur5_needle_insertion` ROS 2 workspace for UR5 robot-based needle insertion path planning. The workspace includes multiple ROS 2 packages and a **submodule for UR5 description** from the **Universal Robots ROS 2 Description repository**.

---

## **1. Prerequisites**
Ensure you have the following installed:
- **ROS 2 (Humble)** → [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Colcon** (for building ROS 2 workspaces)
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

- **Submodule Repository:** [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- **Path in Workspace:** `~/ros2_ws/src/ur5_needle_insertion/Universal_Robots_ROS2_Description`

This package provides the **URDF** and **robot description** files for the UR5 robot.

---

## **6. Running a ROS 2 Node**
To run a specific node, use:
```bash
ros2 run <package_name> <node_name>
```

For example, if you have a node in `ur5_insertion_controller`:
```bash
ros2 run ur5_insertion_controller needle_insertion_node
```

To launch the UR5 robot description:
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

---

## **7. Updating the Repository**
To update your repository and pull changes from GitHub:
```bash
cd ~/ros2_ws/src/ur5_needle_insertion
git pull origin main  # Or replace 'main' with your active branch
git submodule update --recursive --remote
```

If the submodule **UR5 Description** has new changes:
```bash
cd ~/ros2_ws/src/ur5_needle_insertion/Universal_Robots_ROS2_Description
git pull origin main  # Fetch the latest updates from the upstream repo
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
