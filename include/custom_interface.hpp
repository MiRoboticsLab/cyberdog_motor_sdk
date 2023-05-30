// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROJECT_CUSTOMINTERFACE_H
#define PROJECT_CUSTOMINTERFACE_H

#include "leg_control_data_lcmt.hpp"
#include "motor_ctrl_lcmt.hpp"
#include "motor_ctrl_state_lcmt.hpp"
#include "spi_data_t.hpp"
#include "state_estimator_lcmt.hpp"
#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

struct RobotData {
    float q[ 12 ];                 // 12个关节电机角度，弧度制，0:右前腿横滚髋关节，1:右前腿大腿关节
                                   // 2:右前腿膝关节 3/4/5:左前腿  6/7/8:右后腿  9/10/11
    float    qd[ 12 ];             //电机角速度，弧度制
    float    tau[ 12 ];            //电机扭矩 N.M
    float    quat[ 4 ];            //机身姿态四元数 xyzW，右手坐标系
    float    rpy[ 3 ];             //机身姿态横滚、俯仰、偏航角 弧度制
    float    acc[ 3 ];             //加速度计值
    float    omega[ 3 ];           //角速度计值
    float    ctrl_topic_interval;  //控制帧接收时间间隔
    uint32_t motor_flags[ 12 ];    //电机模式和报错信息
    int16_t  err_flag;
};
struct MotorCmd {
    //期望扭矩tau = tau_des + (q_des - q)*kp_des + (qd_des - qd)*kd_des
    float q_des[ 12 ];    // 12个关机电机期望角度，弧度制  0/3/6/9:-0.68~0.68  1/4:2.79
                          // ~ -1.32  7/10:3.14 ~ -0.97  2/5/8/11:-0.52 ~ -2.53
    float qd_des[ 12 ];   //电机期望角速度，弧度制  +-12弧度/秒@24NM
    float kp_des[ 12 ];   //电机位置控制比例系数 0~200
    float kd_des[ 12 ];   //电机速度控制比例系数 0~10
    float tau_des[ 12 ];  //电机期望前馈扭矩  +-12NM
};

class CustomInterface {
public:
    CustomInterface( const double& loop_rate );
    void Spin();
    void Stop();
    void Zero_Cmd( MotorCmd& cmd );
    void PrintData( RobotData& data );

protected:
    virtual void UserCode( bool first_run ) = 0;
    RobotData    robot_data_;
    MotorCmd     motor_cmd_;

private:
    double dt_;
    bool   running_;
    bool   all_thread_done_;

    lcm::LCM leg_data_Lcm_;
    lcm::LCM motor_data_Lcm_;
    lcm::LCM motor_ctrl_state_Lcm_;
    lcm::LCM robot_state_Lcm_;
    lcm::LCM motor_ctrl_Lcm_;

    std::thread motor_ctrl_state_LcmThread_;
    std::thread otor_data_LcmThread_;
    std::thread leg_data_LcmThread_;
    std::thread robot_state_LcmThread_;
    std::thread user_code_ControlThread_;

    motor_ctrl_lcmt motor_ctrl_;

    std::string GetLcmUrl_Port( int64_t port, int64_t ttl );

    void Leg_data_LcmThread() {
        while ( running_ ) {
            leg_data_Lcm_.handleTimeout( 1000 );
        }
    }
    void MotorData_LcmThread() {
        while ( running_ ) {
            motor_data_Lcm_.handleTimeout( 1000 );
        }
    }
    void RobotState_LcmThread() {
        while ( running_ ) {
            robot_state_Lcm_.handleTimeout( 1000 );
        }
    }
    void MotorCtrlState_LcmThread() {
        while ( running_ ) {
            motor_ctrl_state_Lcm_.handleTimeout( 1000 );
        }
    }

    void Handle_MotorCtrlState_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_ctrl_state_lcmt* msg );
    void Handle_MotorData_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const spi_data_t* msg );
    void Handle_LegData_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const leg_control_data_lcmt* msg );
    void Handle_RobotState_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg );

    void MotorCmdSend();
    void Control();
};  // CustomInterface
#endif