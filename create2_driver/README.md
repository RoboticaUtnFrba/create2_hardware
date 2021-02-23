# [create2_driver](https://github.com/AutonomyLab/create_robot/tree/foxy)

ROS 2 driver for iRobot's Create 2.

[ROS 2](http://ros.org) driver for iRobot [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).
This package wraps the C++ library [libcreate][libcreate], which uses iRobot's [Open Interface Specification][oi_spec].

* ROS wiki page: http://wiki.ros.org/create_robot
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))

## Install

#### Prerequisites

* Internet connection
* [ROS](https://index.ros.org/doc/ros2/Installation) _Foxy_
* Ubuntu packages: `python3-rosdep`, `python3-colcon-common-extensions`

``` bash
$ sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Compiling

1. Create a colcon workspace

    ``` bash
    $ cd ~
    $ mkdir -p create_ws/src  
    $ cd create_ws
    ```

2. Clone this repo

    ``` bash
    $ cd ~/create_ws/src
    $ git clone https://github.com/autonomylab/create_robot.git
    ```
  
3. Install dependencies

    ``` bash
    $ cd ~/create_ws
    $ rosdep update  
    $ rosdep install --from-paths src -i  
    ```

4. Build

    ``` bash
    $ cd ~/create_ws
    $ colcon build
    ```
#### USB Permissions

5. In order to connect to Create over USB, ensure your user is in the dialout group
    ``` bash
    $ sudo usermod -a -G dialout $USER
    ```

6. Logout and login for permission to take effect

## Running the driver

### Setup

1. After compiling from source, don't forget to source your workspace:  
    ``` bash
    $ source ~/create_ws/install/setup.bash
    ```

2. Connect computer to Create's 7-pin serial port

3. Launch one of the existing launch files or adapt them to create your own.

### Launch files

For Create 2 (Roomba 600/700 series):
``` bash
$ ros2 launch create_bringup create_2.launch
```

### Teleoperation

`create_bringup` comes with a launch file for teleoperating Create with a joystick.

``` bash
$ ros2 launch create_bringup joy_teleop.launch [joy_config:=xbox360]
```

There exists configuration files for the [Xbox 360 wired controller](https://www.amazon.ca/Microsoft-Xbox-360-Wired-Controller/dp/B003ZSN600) and the [Logitech F710 controller](http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad). You can adapt these files for your preferred joystick configuration.

### Contributors

* [Michael Browne](http://brownem.engineer/)
    - Confirms driver works with Roomba 700 and 800 series.
* [Clyde McQueen](https://github.com/clydemcqueen)
    - Added support for sound ([#37](https://github.com/AutonomyLab/create_autonomy/pull/37)).
* [Ben Wolsieffer](https://github.com/lopsided98) 
    - Added JointState publisher for wheels ([#26](https://github.com/AutonomyLab/create_autonomy/pull/26)).
    - Added Create 1 description ([#27](https://github.com/AutonomyLab/create_autonomy/pull/27)).
* [Pedro Grojsgold](https://github.com/pgold)
    - Ported to ROS 2 ([commit](https://github.com/AutonomyLab/create_robot/commit/198345071aa8a9df154d8490feabf5784b78da16)).

[libcreate]:  https://github.com/AutonomyLab/libcreate
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
