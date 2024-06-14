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