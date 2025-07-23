# 🤖 ws_three_wheel_robot

A ROS Noetic workspace containing a custom 3-wheeled differential drive robot with IMU integration, RViz visualization, and basic velocity control.

---

## 📁 Project Structure

```

ws\_three\_wheel\_robot/
├── src/
│   └── three\_wheel\_robot/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   ├── bringup.launch
│       │   └── imu\_test.launch
│       ├── rviz/
│       │   └── view\.rviz
│       ├── scripts/
│       │   ├── velocity\_publisher.py
│       │   └── parse\_xacro.sh
│       ├── src/
│       └── urdf/
│           └── robot.xacro

````

---

## 🚀 Getting Started

### ✅ Prerequisites

Ensure you are using **ROS Noetic** on **Ubuntu 20.04**.

Install required ROS packages:

```bash
sudo apt update
sudo apt install ros-noetic-xacro \
                 ros-noetic-joint-state-publisher-gui \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-rviz
````

---

### 🔧 Build the Workspace

```bash
cd ~/ws_three_wheel_robot
catkin_make
source devel/setup.bash
```

---

## 🧪 How to Launch

### ▶️ 1. Bring up the robot (URDF + TF + RViz)

```bash
roslaunch three_wheel_robot bringup.launch
```

This will load the robot in RViz with its URDF and display frames.

---

### 📈 2. Test the IMU Launch

```bash
roslaunch three_wheel_robot imu_test.launch
```

Use this to test IMU TFs or sensor messages if applicable.

---

### ⏩ 3. Publish Velocity to Move the Robot

```bash
rosrun three_wheel_robot velocity_publisher.py
```

You can edit the script to change the `linear` and `angular` velocity values.

---

## ⚙️ Scripts

### `velocity_publisher.py`

Publishes to `/cmd_vel` topic using a `Twist` message.

### `parse_xacro.sh`

Simple script to convert `.xacro` → `.urdf` (if needed for debugging)

```bash
bash scripts/parse_xacro.sh
```

---

## 🌐 Visualization

### Open RViz Manually:

```bash
rviz -d $(rospack find three_wheel_robot)/rviz/view.rviz
```

---

## 📝 TODO / Future Work

* Add simulated IMU and wheel odometry
* Integrate with `teleop_twist_keyboard`
* Connect to a differential drive controller
* Add sensors (e.g., LiDAR, depth camera)

---

## 📜 License

This project is open-source and available under the [MIT License](LICENSE).

---

## 👤 Author

**Mahboob Alam**
GitHub: [MahboobXworld](https://github.com/MahboobXworld)

---

## 🌟 Give a Star!

If you find this project helpful, please ⭐️ the repo to show support!

````

