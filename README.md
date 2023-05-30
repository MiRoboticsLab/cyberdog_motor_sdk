CYBERDOG MOTOR SDK
---
此SDK开放了电机驱动器和机身IMU传感器接口，基于**LCM**（**Lightweight Communications and Marshalling**）实现，方便用户进行运动控制的二次开发。具体接口使用可参照cyberdog_motor_sdk中CustomInterface类和CustomCtrl类，按如下步骤在实际机器人上部署运行。

## 1. 准备工作
### 1.1 通信框架
机器人整体的通信框架如下图所示：

<p align="center">
  <img width="600" src="./docs/flow_chart.png">
</p>

运控程序部署在M813运控板上，基于LCM通信提供高层和底层两种接口，供用户调用。用户程序既可以部署在机器人内置的NX应用板上，也可以部署在外部的用户PC上，此时外部PC需通过网口或者USB口连接机器人。详细连接教程参见1.4节连接机器人部分。
### 1.2 LCM介绍
LCM是致力于高带宽，低延时的实时通信开源库，支持多种编程语言。如果需要在用户电脑使用二次开发接口，可以按照如下方法安装LCM：
```shell
# python版安装：pip install lcm

# 源码安装：
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```
此外，LCM库附带了命令行工具lcm-gen，可以生成各种编程语言对应的数据类型定义文件，具体用法如下表：

| 编程语言 | 用法 | 编程语言 | 用法 |
| ------ | ------ | ------ | ------ |
| C    | lcm-gen -c example_t.lcm  | Python | lcm-gen -p example_t.lcm |
| C++  | lcm-gen -x example_t.lcm  | C#     | lcm-gen --csharp example_t.lcm |
| Java | lcm-gen -j example_t.lcm  | MATLAB | Generate Java code |
| Lua  | lcm-gen -l example_t.lcm  | Go     | lcm-gen -g example_t.lcm |

其中，example_t.lcm示例如下：
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
更多使用细节，参见[官方说明文档](https://lcm-proj.github.io/)。

### 1.3 坐标系和关节
机器人坐标系如下图(a)所示，遵循右手法则，其中，x轴指向机器人前方，y轴指向机器人左侧，z轴指向机器人上方。四条腿分别以FR（Front-right）、FL（Front-left）、RR（Rear-right）、RL（Rear-left）来命名，并以此顺序作为腿的编号顺序。每条腿包含3个关节，分别是侧摆髋关节、前摆髋关节、膝关节。关节零位定义见下图(b)，侧摆髋关节水平，前摆髋关节和膝关节组成的腿部垂直向下时为电气零位（因连杆机械限制，无法实际达到）。关节旋转正方向也符合围绕转轴的右手法则，关节范围、最大速度和力矩如下表所示。

<p align="center">
  <img width="800" src="./docs/coordinate.png">
</p>

| 关节名称 | 关节范围 | 最大速度 | 最大力矩 | 关节名称 | 关节范围 | 最大速度 | 最大力矩 |
| ------ | ------- | ------- | ------- | ------ | ------- | ------- | ------- |
| FR-侧摆髋关节 | [-0.68, 0.68] | 38.19 | 12 | FL-侧摆髋关节 | [-0.68, 0.68] | 38.19 | 12 |
| FR-前摆髋关节 | [2.79, -1.33] | 38.19 | 12 | FL-前摆髋关节 | [2.79, -1.33] | 38.19 | 12 |
| FR-膝关节    | [-0.52, -2.53] | 38.19 | 12 | FL-膝关节    | [-0.52, -2.53] | 38.19 | 12 |
| RR-侧摆髋关节 | [-0.68, 0.68] | 38.19 | 12 | RL-侧摆髋关节 | [-0.68, 0.68] | 38.19 | 12 |
| RR-前摆髋关节 | [3.14, -0.98] | 38.19 | 12 | RL-前摆髋关节 | [3.14, -0.98] | 38.19 | 12 |
| RR-膝关节    | [-0.52, -2.53] | 38.19 | 12 | RL-膝关节    | [-0.52, -2.53] | 38.19 | 12 |

### 1.4 连接机器人
使用运动控制二次开发接口，首先需要申请开发者权限，否则相关端口会被封禁，用户无法进入或连接机器人控制板。暂不支持动态切换APP控制和MotorSdk控制，避免控制信号冲突。另外，如果开启了低功耗模式(默认关闭，详见APP设置)，机器人处于趴下状态超过30秒，系统会进入休眠，此时，运动控制接口也无法使用。在确认权限和模式正确后，拆开调试盖板，用户电脑有两种方式连接到机器人控制板：

**方式1(推荐)**：通过网线连接铁蛋的网口，并将用户电脑IP设为：192.168.44.100/255.255.255.0
```shell
$ ping 192.168.44.100     #用户电脑被分配的IP
$ ssh mi@192.168.44.1     #登录NX应用板，密码123
$ ssh root@192.168.44.233 #可从用户电脑直接登录运控板
```
**方式2**：将用户电脑连接至铁蛋的USB Type-C接口(位于中间充电口的右侧)，等待出现”L4T-README” 弹窗，用户电脑IP会被自动分配
```shell
$ ping 192.168.55.100     #用户电脑被分配的IP
$ ssh mi@192.168.55.1     #登录NX应用板 ,密码123
$ ssh root@192.168.44.233 #可在登录NX应用板后，再登录运控板（192.168.44网段）
```
> **注意**：方式2中，用户虽然可以登录NX应用板和运控板查看机器人状态，但因为用户电脑和运控板不在同一个网段，所以用户无法在自己电脑上基于LCM通信使用该运控接口进行开发。

如需**在用户电脑**使用该运控开发接口，按照方式1通过网线连接铁蛋后，在用户电脑用下面脚本配置路由表即可：
```shell
$ ./auto_lcm_init.sh #配置路由表脚本
```

## 2. 底层接口
### 2.1 接口介绍
底层接口的CustomInterface类实现了对电机控制指令的下发，以及关节编码器，IMU传感器信息的获取，具体接口内容包括：
```cpp
/** 反馈信息 **/
struct RobotData {
    float    q[ 12 ];              //12个关节电机角度，弧度制
    float    qd[ 12 ];             //电机角速度，弧度制
    float    tau[ 12 ];            //电机扭矩，单位Nm
    float    quat[ 4 ];            //机身姿态四元数xyzw，右手坐标系
    float    rpy[ 3 ];             //机身姿态横滚、俯仰、偏航角，弧度制
    float    acc[ 3 ];             //IMU加速度计输出，单位m/s^2
    float    omega[ 3 ];           //IMU陀螺仪输出，单位rad/s
    float    ctrl_topic_interval;  //控制帧接收时间间隔
    uint32_t motor_flags[ 12 ];    //电机模式和报错信息，同2.3.1节中的motor_error
    int16_t  err_flag;             //错误标志位
};

/** 控制指令 **/
struct MotorCmd {
    //期望扭矩tau = tau_des + (q_des - q)*kp_des + (qd_des - qd)*kd_des
    float q_des[ 12 ];    //12个关机电机期望角度，弧度制
    float qd_des[ 12 ];   //电机期望角速度，弧度制，-12~12rad/s
    float kp_des[ 12 ];   //电机位置控制比例系数，0~200
    float kd_des[ 12 ];   //电机速度控制比例系数，0~10
    float tau_des[ 12 ];  //电机期望前馈扭矩，-12~12Nm
};
```
RobotData->err_flag错误标志位的具体含义如下：
```markdown
BIT(0) 警告，用户程序和机器人通信延迟超过10ms，为避免危险，关节期望位置和力矩会随时间衰减
BIT(1) 错误，用户程序和机器人通信延迟超过500ms，机器人会进入kp=0, kd=10, tau=0 高阻尼状态趴下，等待用户发全零帧复位
BIT(2) 警告，侧摆髋关节期望位置变化大于8度，该帧变化幅度会被钳位到8度，以避免腿部飞车
BIT(3) 警告，前摆髋关节期望位置变化大于10度，该帧变化幅度会被钳位到10度，以避免腿部飞车
BIT(4) 警告，膝关节期望位置变化大于12度，该帧变化幅度会被钳位到12度，以避免腿部飞车
```
> **注意**：bit2~bit4为**位置指令跳变警告**，即相邻帧期望角度变化过大（比如膝关节前一帧的期望角度是10度，如果当前帧下发30度，实际会被钳位到10度执行）。该机制用于提供一定的飞车保护功能，默认开启。如需关闭，可以登录运控板，将配置文件/robot/robot-software/common/config/cyberdog-mini-ctrl-user-parameters.yaml 中的 motor_sdk_position_mutation_limit 置0。

> **注意**：出现BIT(1)超时错误时，待机器人趴下后(2s)发送全零控制帧可以清除错误。由于大部分超时是通信断联引起的，此时如果是在用户电脑上部署运行，需要重新配置路由表。

### 2.2 控制模式切换
成功连接机器人并验证登录后（详见1.4 连接机器人），向运控发布消息名为"motor_ctrl"的Motor_Cmd结构体LCM消息命令，可使运控板进入电机控制模式，其中前几帧需为全零帧，用于激活机器人并复位通信超时错误，详见SDK中CustomInterface类初始化示例。

如需退出电机控制模式，重启机器人即可，此时会恢复至普通APP控制模式。

使用底层控制接口过程中建议断开APP连接，避免控制指令冲突。

### 2.3 编译及部署

#### 2.3.1 用户电脑部署
使用底层接口的程序部署在用户电脑，难以保证lcm通信的实时性，可能会影响控制性能，仅推荐编译验证和简单的关节位置控制测试 ：

- 安装LCM通信库（如1.2节所述）
- 通过网口连接机器人，并配置路由表
```shell
$ ./auto_lcm_init.sh #配置路由表，脚本存放在高层接口例程仓库
```
- 编译并运行代码
```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk #下载底层接口示例代码
$ cd cyberdog_motor_sdk
$ mkdir build && cd build
$ cmake .. #编译
$ make -j4
$ ./example_motor_ctrl #运行
```

#### 2.3.2 NX应用板部署
和部署在用户电脑一样，考虑到跨设备通信的时延和不稳定，在NX应用板使用底层接口，也仅推荐编译验证和简单的关节位置控制测试：

```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk #下载底层接口示例代码
$ scp -r cyberdog_motor_sdk mi@192.168.44.1:/home/mi/ #将底层接口示例代码拷入应用板，密码123
$ ssh mi@192.168.44.1 #登录应用板
mi@lubuntu:~$ cd /home/mi/cyberdog_motor_sdk
mi@lubuntu:~$ mkdir build && cd build
mi@lubuntu:~$ cmake .. #编译
mi@lubuntu:~$ make -j2
mi@lubuntu:~$ ./example_motor_ctrl #运行
```
#### 2.3.3 运控板交叉编译部署
运控板TinaLinux经过裁剪，缺少编译环境，故运控侧部署需在用户电脑上使用docker镜像环境交叉编译，然后再将编译产物发送至运控板上运行，具体步骤如下：
- 在用户电脑安装docker(https://docs.docker.com/engine/install/ubuntu/)，并给docker设置root权限：
```shell
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```
- 下载交叉编译所需docker镜像
```shell
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```
- 运行docker镜像进行交叉编译
```shell
$ git clone -b mini_cyberdog https://github.com/MiRoboticsLab/cyberdog_motor_sdk #下载底层接口示例代码
$ docker run -it --rm --name cyberdog_motor_sdk -v /home/xxx/{sdk_path}:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash #运行docker镜像，/home/xxx/{sdk_path}是示例代码绝对路径
[root:/work] # cd /work/build_farm/workspace/cyberdog/ #进入docker系统的代码仓
[root:/work/build_farm/workspace/cyberdog] # mkdir onboard-build && cd onboard-build
[root:/work/build_farm/workspace/cyberdog] # cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake .. #指定交叉编译工具链
[root:/work/build_farm/workspace/cyberdog] # make -j4 #编译
[root:/work/build_farm/workspace/cyberdog] # exit #退出docker镜像
```
- 将编译产物libcyber_dog_sdk.so和example_motor_ctrl拷贝至运控板/mnt/UDISK目录下运行
```shell
$ cd /home/xxx/{sdk_path}/onboard-build
$ ssh root@192.168.44.233 "mkdir /mnt/UDISK/cyberdog_motor_sdk" #在运控板内创建文件夹
$ scp libcyber_dog_motor_sdk.so example_motor_ctrl root@192.168.44.233:/mnt/UDISK/cyberdog_motor_sdk #拷贝编译产物
$ ssh root@192.168.44.233 #进入运控板
root@TinaLinux:~# cd /mnt/UDISK/cyberdog_motor_sdk
root@TinaLinux:~# export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk #设置动态链接库路径变量
root@TinaLinux:~# ./example_motor_ctrl #运行，也可通过“nohup ./example_motor_ctrl &”后台运行，退出ssh连接不受影响
```
- 如需开机自启动，可配置/robot/robot-software/common/config/fork_para_conf_lists.json 进程管理文件(注意结尾逗号)，然后重启运控程序或者重启运控板
 "600003": {"fork_config":{"name": "example_motor_ctrl",  "object_path": "/cyberdog_motor_sdk/",  "log_path": "", "paraValues": ["", "", ""] }}
```shell
# 重启运控程序:
$ ssh root@192.168.44.233 "ps | grep -E 'example_motor_ctrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" #需先于主进程暂停，避免急停
$ ssh root@192.168.44.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
$ ssh root@192.168.44.233 "export LD_LIBRARY_PATH=/robot/robot-software/build;/robot/robot-software/manager /robot/ >> /robot/manager.log 2>&1 &"

# 重启运控板系统:
$ ssh root@192.168.44.233 "reboot"
```

### 2.4 接口示例
底层接口代码中包含了一个示例程序example_motor_ctrl.cpp，它实现了机器人从当前实际关节位置线性插值到站立姿态，从而实现基本的站立功能。

其中first_run在起始和错误出现时被置True，用于进行复位初始化。相关错误重置逻辑可自行按需调整。

![](./docs/motor_sdk_flow_chart_cn.png)

target1_q和target2_q分别对应蹲下和站立时的单腿关节角，角度正方向符合右手坐标系，详见1.3节坐标系和关节。

> **注意**：进行关节位控开发调试时，推荐将机器人悬起，关节kp_des 和 kd_des 从小值起调，如(5/0.2)，出现异常，及时关闭相关进程，过程中谨慎操作避免夹手。
> **注意**：电机控制模式通信超时后，运控板会接管控制机器人趴下，并停止响应后续指令，log提示：Err: 0x02 Communicate lost over 500ms。此时可以通过发送全零控制帧复位。电机控制模式初始化时默认为通信超时状态，避免遗留进程误发位置指令导致危险，故每次开机需先发全零帧激活。
