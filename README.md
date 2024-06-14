# About

[DIAGRAM HERE]

This repository is for the **ECU-HeadUnit** part of the [Autonomous-Driving-System](https://github.com/SEA-ME-COSS/Autonomous-Driving-System) project. The ECU-HeadUnit is based on an independent RaspberryPi board and displays information such as vehicle's driving status and location on the head unit screen. The head unit was developed with QT5 and interacts with [ECU-Core](https://github.com/SEA-ME-COSS/ECU-Core) via CAN communication.

<div width="100%" align="center">
    <img width="49%" src="/images/gps_off.png">
    <img width="49%" src="/images/gps_on.png">
</div>

This repository includes head unit developed in **Ubuntu** OS. By following the documentation, you can setup the environment and run the head unit on Ubuntu. However, in the overall project, this head unit runs in an OS based on the **Yocto Project** and is updated via **OTA**. For more detailed information, refer to the [Autonomous-Driving-System](https://github.com/SEA-ME-COSS/Autonomous-Driving-System) project.

# Requirements

- **Ubuntu 20.04**

    Install Ubuntu 20.04 for RaspberryPi using RaspberryPi OS Imager.

- **CAN HAT setup**

    Follow the instruction of [2-CH CAN FD HAT setup](https://www.waveshare.com/wiki/2-CH_CAN_FD_HAT) and enable **Single SPI Mode**.

- **HDMI display setup**

    Follow the instruction of [7inch HDMI LCD setup](https://www.waveshare.com/wiki/7inch_HDMI_LCD_(H)_(with_case)).

- **QT packages**

    ```bash
    sudo apt install qt5-default
    sudo apt install qtdeclarative5-dev
    sudo apt install qml-module-qtquick-controls
    sudo apt install qml-module-qtquick-extras
    sudo apt install libqt5serialbus5*
    ```

# Usage

```bash
# Execute on the ECU-HeadUnit
mkdir build && cd build
cmake ..
make

cd ..
sh can_setup.sh
sh run.sh
```

# Note

Context of CAN communication

| Message            | Purpose                | Arbitration ID |
|--------------------|------------------------|----------------|
| **steering**       | **Control**            | **0x00**       |
| **throttle**       | **Control**            | **0x01**       |
| **x position**     | **GPS**                | **0x02**       |
| **y position**     | **GPS**                | **0x03**       |
| **orientation**    | **GPS**                | **0x04**       |
| **headunit start** | **Autonomous driving** | **0x05**       |

# Reference
- [Head Unit Design](https://github.com/SEA-ME-COSS/In-Vehicle-Infotainment)












=================



# Localization

This project will cover the following contents.

| Type              | Content & Specification | Functionality            |
|-------------------|-------------------------|--------------------------|
| **H/W Sensor**    | LiDAR (YDLIDAR X4)      | Scan & Localization      |
| **H/W Sensor**    | RGBD (RealSense D435)   | Vision                   |

## LiDAR code language performance (CPU)

| Machine                     | C++ (%) | Python (%) |
|-----------------------------|---------|------------|
| **Laptop/Ubuntu20.04**      | 1.11    | 1.87       |
| **RaspberryPi/Ubuntu20.04** | 2.51    | 5.44       |

LiDAR max rotation = 8.3~8.4rps (H/W specific)

## Command note
```bash
# enable lidar connection
sudo chmod a+rw /dev/ttyUSB0  

# enable inter-machine connection
sudo ufw disable

# enable controller using ssh connection
ssh team5@192.168.1.150 -Y
ssh team5@192.168.0.110 -Y
pw = ' '

# launch slam toolbox
cd /opt/ros/foxy/share/slam_toolbox/config
ros2 launch slam_toolbox localization_launch.py

# check camera connection
realsense-viewer

# configure WLAN connection in RPi
sudo nano /etc/netplan/50-cloud-init.yaml
sudo netplan apply

# Bring the 'can0' interface up and set its bitrate to 500,000 bits per second
sudo ip link set can0 up type can bitrate 500000

# Set the transmit queue length of 'can0' to 65,536 (maximum value)
sudo ifconfig can0 txqueuelen 65536

ssh team5@192.168.0.127

cansend can0 000#11.22.33.44
candump can0

pip install transforms3d
```

## ROS2 msg type
- [TFMessage](https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)
- [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

## References

- [YDLIDAR](https://github.com/YDLIDAR)
- [RealSense Doc](https://dev.intelrealsense.com/docs/docs-get-started)
- [pyrealsense2 setup for RPi](https://github.com/IntelRealSense/librealsense/issues/12462)
- [laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
- [map editor](https://github.com/TheOnceAndFutureSmalltalker/ros_map_editor)