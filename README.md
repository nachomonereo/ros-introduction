# ROS Introduction
Fork or git clone this repository

## Before build the docker
add this into your bashrc
```
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```
## How to build the container
To build the image  
```
.docker/build_image.sh
```
To run the image
```
.docker/run_user.sh
```
You may need to change the owner of the dev_ws, copy the line showing on the terminal
```
sudo chown -R [YOUR USER NAME] /dev_ws
```
Start a terminal
```
terminator
```

# 🐢 ROS 1 - Turtlesim Star Drawing

This repository contains a ROS 1 script that allows a turtle in `turtlesim` to draw a star using `ROS Noetic`.

## 📌 **Prerequisites**

Before starting, make sure you have the following:
- **Docker** installed and running.
- **ROS Noetic** configured inside a Docker container.
- **Git** installed to clone this repository.

## 📥 **Installation and Setup**

### 1️⃣ **Clone the Repository**
```bash
git clone git@github.com:nachomonereo/ros-introduction.git
cd ros-introduction
```

### 2️⃣ **Set Up the Environment**
#### **Run Docker and Create the Container**
```bash
docker run -it --name ros1_noetic --rm osrf/ros:noetic-desktop-full bash
```
This will launch a ROS Noetic container.

#### **Create the Catkin Workspace**
```bash
mkdir -p /dev_ws/src
cd /dev_ws
catkin_make
source devel/setup.bash
```

### 3️⃣ **Install `turtlesim` and Dependencies**
```bash
apt update
apt install ros-noetic-turtlesim -y
```

### 4️⃣ **Verify `turtlesim` is Working**
```bash
roscore &
rosrun turtlesim turtlesim_node
```

## 🖍️ **Draw the Star with `turtlesim`**

### 5️⃣ **Create the Package and Script**
```bash
cd /dev_ws/src
catkin_create_pkg turtle_star rospy std_msgs geometry_msgs
mkdir -p turtle_star/scripts
```

Create the script `draw_star.py` inside the `scripts/` folder and add the following code:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math

def move_turtle(pub, linear, angular, duration):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

def draw_star():
    rospy.init_node('turtle_star', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)

    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        teleport(5.5, 5.5, 0)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error teleporting: {e}")

    rospy.sleep(1)

    side_length = 2.0
    angle = 144

    for _ in range(5):
        move_turtle(pub, side_length, 0, 1.0)
        move_turtle(pub, 0, math.radians(angle), 1.0)

    rospy.loginfo("Star drawing completed.")

if __name__ == '__main__':
    try:
        draw_star()
    except rospy.ROSInterruptException:
        pass
```

### 6️⃣ **Make the Script Executable**
```bash
chmod +x /dev_ws/src/turtle_star/scripts/draw_star.py
```

### 7️⃣ **Compile and Run the Script**
```bash
cd /dev_ws
catkin_make
source devel/setup.bash
rosrun turtle_star draw_star.py
```

## 📷 **Screenshot**
The image `turtle_star.png` inside the repository shows the final result.

## 🎓 **Credits**
This project was developed with contributions from:
- **IAAC - Institute for Advanced Architecture of Catalonia**
- **ChatGPT**

## 🚀 **Contributions**
If you have improvements or suggestions, feel free to open a pull request! 🎉




