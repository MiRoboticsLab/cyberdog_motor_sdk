CYBERDOG MOTOR SDK
---
This SDK provides the interface of joint motors and IMU sensor mounted on CyberDog, which is build based on **LCM**（**Lightweight Communications and Marshalling**）communication protocol. Users can use this interface to develop their own controllers more freely
. To deploy on real robots, please follow the following steps.

## Preparatory work
### 1.1 Communication Framework

The overall communication framework of the robot is shown in the following figure:

<p align="center">
  <img width="600" src="./docs/flow_chart_en.png">
</p>

The motion control program is deployed on the M813 motion control board, which provides high-level and low-level interfaces based on LCM communication for developers to call. The developer's user program can be deployed on the robot's built-in NX application board, or on an external developer's PC. In the latter case, the external PC needs to be connected to the robot via Ethernet or USB. For detailed connection instructions, please refer to the section on connecting the robot (section 1.4).

### 1.2 Introduction to LCM

LCM is an open source library dedicated to real-time communication with high bandwidth and low latency, supporting multiple programming languages. If you want to use the secondary development interface on your computer, you can install LCM as follows:

```shell
# Method 1: install via pip
$ pip install lcm

# Method 2: source code installation
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```

In addition, the LCM library comes with a command-line tool, lcm-gen, which can generate data type definition files corresponding to various programming languages. The usage is shown in the table below:

| Programming language | Usage | Programming language | Usage |
| ------ | ------ | ------ | ------ |
| C    | lcm-gen -c example_t.lcm  | Python | lcm-gen -p example_t.lcm |
| C++  | lcm-gen -x example_t.lcm  | C#     | lcm-gen --csharp example_t.lcm |
| Java | lcm-gen -j example_t.lcm  | MATLAB | Generate Java code |
| Lua  | lcm-gen -l example_t.lcm  | Go     | lcm-gen -g example_t.lcm |

Here is an example of `example_t.lcm`:

```cpp
struct example_t
{
    int64_t  timestamp;
    double   position[3];
    double   orientation[4]; 
    int32_t  num_ranges;
    int16_t  ranges[num_ranges];
    string   name;
    boolean  enabled;
}
```
For more details on usage, please refer to the [official documentation](https://lcm-proj.github.io/).

### 1.3 Coordinate System and Joints

The robot coordinate system is shown in the figure (a) below, following the right-hand rule, where the x-axis points forward, the y-axis points to the left, and the z-axis points upward. The four legs are named FR (Front-right), FL (Front-left), RR (Rear-right), and RL (Rear-left), and are numbered in this order. Each leg contains three joints: the abad joint, the hip joint, and the knee joint. The definition of the joint zero position is shown in the figure (b) below. The abad joint is horizontal, and the hip joint and knee joint constitute the electrical zero position when the leg is vertically downward (which cannot be actually achieved due to mechanical limitations of the links).

<p align="center">
  <img width="800" src="./docs/coordinate_en.png">
</p>

The positive rotation direction of the joints follows the right-hand rule around the rotation axis. The joint range, maximum speed and torque are shown in the table below.

| Joint name | Joint range | Maximum speed | Maximum torque | Joint name | Joint range | Maximum speed | Maximum torque |
| ------- | -------------- | ----- | -- | ------- | -------------- | ----- | -- |
| FR-abad | [-0.68, 0.68]  | 38.19 | 12 | FL-abad | [-0.68, 0.68]  | 38.19 | 12 |
| FR-hip  | [2.79, -1.33]  | 38.19 | 12 | FL-hip  | [2.79, -1.33]  | 38.19 | 12 |
| FR-knee | [-0.52, -2.53] | 38.19 | 12 | FL-knee | [-0.52, -2.53] | 38.19 | 12 |
| RR-abad | [-0.68, 0.68]  | 38.19 | 12 | RL-abad | [-0.68, 0.68]  | 38.19 | 12 |
| RR-hip  | [3.14, -0.98]  | 38.19 | 12 | RL-hip  | [3.14, -0.98]  | 38.19 | 12 |
| RR-knee | [-0.52, -2.53] | 38.19 | 12 | RL-knee | [-0.52, -2.53] | 38.19 | 12 |

### 1.4 Connect Robot

To use the secondary development interface, you first need to apply for developer permissions. Otherwise, the relevant ports will be blocked and developers will not be able to enter or connect to the robot control board. Currently, it does not support dynamic switching between APP control and MotorSdk control to avoid control signal conflicts. In addition, if the low-power mode is enabled (disabled by default, see APP settings for details), the robot will enter sleep mode after lying down for more than 30 seconds and the motion control interface will also be unavailable. After confirming the correct authority and mode, remove the debugging cover on the robot back. And then, the developer's computer can be connected to the robot control board in two ways:

**Method 1 (recommended)**: Connect the robot's Ethernet port to the developer's computer using a network cable, and set the developer's computer IP address to `192.168.44.100/255.255.255.0`.

```shell
$ ping 192.168.44.100      # IP address assigned to the developer's computer
$ ssh mi@192.168.44.1      # Login to the NX application board with password 123
$ ssh root@192.168.44.233  # Can directly login to the motion control board from the developer's computer
```

**Method 2**: Connect the developer's computer to the robot's USB Type-C port (located on the right side of the middle charging port), and wait for the "L4T-README" pop-up window to appear. The developer's computer IP address will be automatically assigned.

```shell
$ ping 192.168.55.100      # IP address assigned to the developer's computer
$ ssh mi@192.168.55.1      # Login to the NX application board with password 123
$ ssh root@192.168.44.233  # After logging into the NX application board, login to the motion control board (192.168.44 network segment)
```

> **_Note_**: In method 2, although the developer can log in to the NX application board and the motion control board to check the robot's status, the developer cannot use the motion control interface based on LCM communication for development, since the developer's computer and the robot's motion control board are not on the same network segment.

If you want to use this motion control interface on your own computer, then please connect to the robot's Ethernet port as suggested in Method 1 and run the following script to configure the routing table of your computer:
```shell
$ ./auto_lcm_init.sh # script to configure routing table 
```

## 2. Low-level Interface
### 2.1 Introduction to Low-level Interface
The CustomInterface class realizes the sending of low-level control command and the acquisition of encoders and IMU sensory feedback. The interface contains two struct:
```c++
/** Sensory Feedback Struct **/
struct RobotData {
    float    q[ 12 ];              // joint angles in rad
    float    qd[ 12 ];             // joint angular velocities in rad/s
    float    tau[ 12 ];            // motor torque in Nm
    float    quat[ 4 ];            // quaternion of robot in xyzw
    float    rpy[ 3 ];             // roll pitch yaw in rad
    float    acc[ 3 ];             // IMU acceleration output in m/s^2
    float    omega[ 3 ];           // IMU angular velocity output in rad/s
    float    ctrl_topic_interval;  // control topic interval
    uint32_t motor_flags[ 12 ];    // motor error flag，as motor_error in 2.3.1
    int16_t  err_flag;             // error flag
};

/** Motor Control Struct **/
struct MotorCmd {
    //tau = tau_des + (q_des - q)*kp_des + (qd_des - qd)*kd_des
    float q_des[ 12 ];    //desired joint angles in rad
    float qd_des[ 12 ];   //desired joint velocities in rad，-12~12rad/s
    float kp_des[ 12 ];   //position gain，0~200
    float kd_des[ 12 ];   //velocity gain，0~10
    float tau_des[ 12 ];  //desired feedforward torque，-12~12Nm
};
```
The meanings of each bit of error flag `RobotData->err_flag` are listed below:


- BIT(0) WARNING, the communication delay between user program and robot exceeds 10ms, the desired joint position and desired joint torque will decay with time to avoid danger
- BIT(1) ERROR, the communication delay between the user program and the robot exceeds 500ms, the robot will enter the high damping mode(kp=0, kd=10, tau=0) and lie down, waiting for all-zero frames to reset
- BIT(2) WARNING, the desired position of abad joint changes by more than 8 degrees, the change range will be clamped to 8 degrees to avoid danger
- BIT(3) WARNING, the desired position change of hip joint changes by more than 10 degrees, and the change range will be clamped to 10 degrees to avoid danger
- BIT(4) WARNING, the desired position change of knee joint changes by more than 12 degrees, and the change range will be clamped to 12 degrees to avoid danger


> **_NOTE:_** BIT(2)~ BIT(4) are **position jump warnings**, i.e. the desired joint positions of adjacent frames changes too much (e.g., the desired positions of the knee joint is 10 degrees in the previous frame and it is 30 degrees in current frame, then the actual position change will be clamped to 10 degrees, i.e. the actual current desired position is 20 deg). This mechanism is used to prevent violent leg movements and is by default **turned on**, to close it, you can ssh into the MC board and change the configuration bit `motor_sdk_position_mutation_limit` in `/robot/robot-software/common/config/cyberdog-mini-ctrl-user-parameters.yaml` from 1 to 0.

> **_NOTE:_** BIT(1) error can be cleared when user sends all-zero low-level commands after the robot lying down for 2s. In most cases, BIT(1) error is caused by communication breakdown,  if the control program is running from an external PC, user may need to reconfigure the route table after reconnection.

### 2.2 Control Mode Switching 
To switch control mode of MC board to **low-level control mode** , user need to send a few frames of all-zero low-level control command (as defined in Motor_Cmd struct) to the MC board through the LCM topic named "motor_ctrl" after successfully connecting the robot and verifying the login (see 1.4 Connecting the Robot for details).  The all-zero frames are used to activate the low-level control mode and clear the communication error bits, see the initialization step of CustomInterface class in the SDK for details.

To exit the low-level control mode, restart the robot. It will return to the normal APP control mode after the reboot.

It is recommended to disconnect the APP during the use of low-level interface to avoid control command conflicts.

### 2.3 Source Code Compilation and Deployment
#### 2.3.1 Deployment on External PC
If the robot controller is running on an external PC, it is hard to guarantee the realtime communication through LCM, the actual performance may be degenerated due to latency. It is recommended that users only do verification tests or simple joint control tasks when sending control commands from an external PC. 

1. Install LCM（following 1.2）
2. Connect the robot through ethernet cable and configure the routing table. 

```shell
$ ./auto_lcm_init.sh #configure routing table, the script is stored in the repository of high-level exampel code
```

3. Build the source code and run the example

```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk #clone the motor sdk example code
$ cd cyberdog_motor_sdk
$ mkdir build && cd build
$ cmake .. #build the example code
$ make -j4
$ ./example_motor_ctrl #run the example
```

#### 2.3.2 Deployment on NX Board
The procedure is similar to Deployment on an external PC. Due to communication latency, it is recommended that users only do verification tests or simple joint control tasks when sending low-level control commands from NX boards. 


```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk #clone the motor sdk example code
$ scp -r cyberdog_motor_sdk mi@192.168.44.1:/home/mi/ #copy the example code to NX board, default password is 123
$ ssh mi@192.168.44.1 #login to NX board
mi@lubuntu:~$ cd /home/mi/cyberdog_motor_sdk
mi@lubuntu:~$ mkdir build && cd build
mi@lubuntu:~$ cmake .. #build the example code
mi@lubuntu:~$ make -j2
mi@lubuntu:~$ ./example_motor_ctrl #run the example
```

#### 2.3.3 Cross-compilation and Deployment on MC Board 
The TinaLinux on MC board is tailored and does not contain the necessary compiling envrionment. Users need to cross-compile the source code in the  docker environment in order to run the compiled code on MC board. 

To cross-compile the source code from docker, follow the instruction below:
1. Install docker(https://docs.docker.com/engine/install/ubuntu/), then add **sudo** privileges to user 

```shell
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```

2. Download the docker image for cross-compilation

```shell
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```

3. Use the docker image to cross build the source code

```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk # clone the low-level interface example code
$ docker run -it --rm --name cyberdog_motor_sdk -v /home/xxx/{sdk_path}:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash #run docker image，/home/xxx/{sdk_path} is the absolute path to the example code folder
[root:/work] # cd /work/build_farm/workspace/cyberdog/ 
[root:/work/build_farm/workspace/cyberdog] # mkdir onboard-build && cd onboard-build
[root:/work/build_farm/workspace/cyberdog] # cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake .. #specify toolchain for cross-compilation
[root:/work/build_farm/workspace/cyberdog] # make -j4 
[root:/work/build_farm/workspace/cyberdog] # exit 
```
4. Copy `libcyber_dog_sdk.so` and `example_motor_ctrl` to MC board under `/mnt/UDISK`.

```shell
$ cd /home/xxx/{sdk_path}/onboard-build
$ ssh root@192.168.44.233 "mkdir /mnt/UDISK/cyberdog_motor_sdk"
$ scp libcyber_dog_motor_sdk.so example_motor_ctrl root@192.168.44.233:/mnt/UDISK/cyberdog_motor_sdk 
$ ssh root@192.168.44.233
root@TinaLinux:~# cd /mnt/UDISK/cyberdog_motor_sdk
root@TinaLinux:~# export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk
root@TinaLinux:~# ./example_motor_ctrl #run the example or use “nohup ./example_motor_ctrl &” to run in the background
```

5. To run the task on startup, add the following line to `/robot/robot-software/common/config/fork_para_conf_lists.json` process management file(do not forget the ',' at the end)，then restart motion control process or reboot to make the configuration take effect.
```  
 "600003": {"fork_config":{"name": "example_motor_ctrl",  "object_path": "/cyberdog_motor_sdk/",  "log_path": "", "paraValues": ["", "", ""] }}
```

```shell
# method1: restart the main motion control process:
$ ssh root@192.168.44.233 "ps | grep -E 'example_motor_ctrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" #must be killed before main control progress to avoid sudden drop of robot.
$ ssh root@192.168.44.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
$ ssh root@192.168.44.233 "export LD_LIBRARY_PATH=/robot/robot-software/build;/robot/robot-software/manager /robot/ >> /robot/manager.log 2>&1 &"

# method2: reboot the system:
$ ssh root@192.168.44.233 "reboot"
```

### 2.4 Low-level Interface Example
We provide an example `example_motor_ctrl.cpp` in the source code, in which we realize the basic standing up function by sending low-level control command, the positon of which is linearly interpolated from current joint position to default stand position.
The `first_run` flag is set to **True** when first run or when an error triggers.

![](./docs/motor_sdk_flow_chart_en.png)

`target1_q` and `target2_q` correspond to default joint positon of sitting and standing, and the definition of positive direction follows the right-handed rule, as detailed in **1.3**. 

> **_NOTE:_** For position control development, it is recommended to suspend the robot, adjust the **kp_des** and **kd_des** from a small value, e.g. kp=5, kd=0.2, to avoid dangerous movement. Be careful during tests as the moving robot could cause harm.

> **_NOTE:_**  The MC board will take control of the robot whenever the communication with motor SDK process times out，and the following error message will show up： `Err: 0x02 Communicate lost over 500ms`. Users need to send a few frames of all-zero low-level control commands to re-activate the low-level control mode and clear the error bits.
Communication timeout error bit is by default triggered after startup to prevent dangerous movement caused by unexpected commands from some unkilled previous control process.
