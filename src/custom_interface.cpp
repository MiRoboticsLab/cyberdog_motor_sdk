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

#include <custom_interface.hpp>

std::string CustomInterface::GetLcmUrl_Port( int64_t port, int64_t ttl ) {
    assert( ttl >= 0 && ttl <= 255 );
    return "udpm://239.255.76.67:" + std::to_string( port ) + "?ttl=" + std::to_string( ttl );
}

CustomInterface::CustomInterface( const double& loop_rate )
    : motor_data_Lcm_( GetLcmUrl_Port( 7667, 255 ) ), motor_ctrl_state_Lcm_( GetLcmUrl_Port( 7667, 255 ) ), robot_state_Lcm_( GetLcmUrl_Port( 7669, 255 ) ),
      motor_ctrl_Lcm_( GetLcmUrl_Port( 7667, 255 ) ), leg_data_Lcm_( GetLcmUrl_Port( 7667, 255 ) ) {

    running_         = true;
    all_thread_done_ = false;

    robot_data_.err_flag |= 0x02;

    if ( loop_rate > 0 ) {
        dt_ = 1.0 / loop_rate;
    }
    else {
        std::cout << "Loop rate should be more than zero! Set to default 500Hz." << std::endl;
        dt_ = 1.0 / 500;
    }

    motor_ctrl_state_Lcm_.subscribe( "motor_ctrl_state", &CustomInterface::Handle_MotorCtrlState_Lcm, this );
    motor_data_Lcm_.subscribe( "spi_data", &CustomInterface::Handle_MotorData_Lcm, this );
    leg_data_Lcm_.subscribe( "leg_control_data", &CustomInterface::Handle_LegData_Lcm, this );
    robot_state_Lcm_.subscribe( "state_estimator", &CustomInterface::Handle_RobotState_Lcm, this );

    motor_ctrl_state_LcmThread_ = std::thread( &CustomInterface::MotorCtrlState_LcmThread, this );
    otor_data_LcmThread_        = std::thread( &CustomInterface::MotorData_LcmThread, this );
    leg_data_LcmThread_         = std::thread( &CustomInterface::Leg_data_LcmThread, this );
    robot_state_LcmThread_      = std::thread( &CustomInterface::RobotState_LcmThread, this );
    user_code_ControlThread_    = std::thread( &CustomInterface::Control, this );
}

void CustomInterface::Control() {
    auto timerFd     = timerfd_create( CLOCK_MONOTONIC, 0 );
    int  seconds     = ( int )dt_;
    int  nanoseconds = ( int )( 1e9 * std::fmod( dt_, 1.f ) );
    int  log_count   = 0;
    bool first_run   = true;

    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec  = seconds;
    timerSpec.it_value.tv_sec     = seconds;
    timerSpec.it_value.tv_nsec    = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;

    timerfd_settime( timerFd, 0, &timerSpec, nullptr );
    unsigned long long missed = 0;
    while ( running_ ) {

        log_count++;

        UserCode( first_run );

        if ( robot_data_.err_flag & 0x02 ) {
            if ( log_count % 1000 == 0 ) {
                printf( "\nErr: 0x02 Communicate lost over 500ms !\n" );
                printf( "Please check the lcm communication and reset motor cmd !!!\n" );
            }
            Zero_Cmd( motor_cmd_ );
            first_run = true;
        }
        else
            first_run = false;

        for ( int i = 0; i < 12; i++ ) {
            if ( robot_data_.motor_flags[ i ] & 0x3FFFFFFF ) {
                printf( "Motor %d error 0x%x! Robot need estip for safty!", i, robot_data_.motor_flags[ i ] );
                running_ = false;
                break;
            }
        }

        MotorCmdSend();

        int m = read( timerFd, &missed, sizeof( missed ) );
        ( void )m;
    }
}

void CustomInterface::Spin() {
    while ( !all_thread_done_ ) {
        sleep( 1.0 );
    }

    printf( "~ Exit ~\n" );
}
void CustomInterface::Zero_Cmd( MotorCmd& cmd ) {
    for ( int i = 0; i < 12; i++ ) {
        cmd.q_des[ i ]   = 0;
        cmd.qd_des[ i ]  = 0;
        cmd.kp_des[ i ]  = 0;
        cmd.kp_des[ i ]  = 0;
        cmd.kd_des[ i ]  = 0;
        cmd.tau_des[ i ] = 0;
    }
}
void CustomInterface::Stop() {
    running_ = false;
    motor_ctrl_state_LcmThread_.join();
    otor_data_LcmThread_.join();
    robot_state_LcmThread_.join();
    user_code_ControlThread_.join();
    all_thread_done_ = true;
}

void CustomInterface::PrintData( RobotData& data ) {
    printf( "interval:---------%.4f-------------\n", data.ctrl_topic_interval );
    printf( "rpy [3]: %.2f %.2f %.2f\n", data.rpy[ 0 ], data.rpy[ 1 ], data.rpy[ 2 ] );
    printf( "acc [3]: %.2f %.2f %.2f\n", data.acc[ 0 ], data.acc[ 1 ], data.acc[ 2 ] );
    printf( "omeg[3]: %.2f %.2f %.2f\n", data.omega[ 0 ], data.omega[ 1 ], data.omega[ 2 ] );
    printf( "quat[4]:" );
    for ( int i = 0; i < 4; i++ )
        printf( " %.2f", data.quat[ i ] );
    printf( "\nq  [12]:" );
    for ( int i = 0; i < 12; i++ )
        printf( " %.2f", data.q[ i ] );
    printf( "\nqd [12]:" );
    for ( int i = 0; i < 12; i++ )
        printf( " %.2f", data.qd[ i ] );
    printf( "\ntau[12]:" );
    for ( int i = 0; i < 12; i++ )
        printf( " %.2f", data.tau[ i ] );
    printf( "\n\n" );
}

void CustomInterface::MotorCmdSend() {
    int sig[ 12 ] = { 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1 };
    for ( int i = 0; i < 12; i++ ) {
        motor_ctrl_.q_des[ i ]   = motor_cmd_.q_des[ i ] * sig[ i ];
        motor_ctrl_.qd_des[ i ]  = motor_cmd_.qd_des[ i ] * sig[ i ];
        motor_ctrl_.kp_des[ i ]  = motor_cmd_.kp_des[ i ];
        motor_ctrl_.kd_des[ i ]  = motor_cmd_.kd_des[ i ];
        motor_ctrl_.tau_des[ i ] = motor_cmd_.tau_des[ i ] * sig[ i ];
    }
    motor_ctrl_Lcm_.publish( "motor_ctrl", &motor_ctrl_ );
}

void CustomInterface::Handle_MotorCtrlState_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_ctrl_state_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    robot_data_.err_flag            = msg->err_flag;
    robot_data_.ctrl_topic_interval = msg->ctrl_topic_interval;
}
void CustomInterface::Handle_MotorData_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const spi_data_t* msg ) {
    ( void )rbuf;
    ( void )chan;
    int sig[ 12 ] = { 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1 };
    for ( int i = 0; i < 4; i++ ) {
        robot_data_.q[ i * 3 + 0 ]   = msg->q_abad[ i ] * sig[ i * 3 + 0 ];
        robot_data_.q[ i * 3 + 1 ]   = msg->q_hip[ i ] * sig[ i * 3 + 1 ];
        robot_data_.q[ i * 3 + 2 ]   = msg->q_knee[ i ] * sig[ i * 3 + 2 ];
        robot_data_.qd[ i * 3 + 0 ]  = msg->qd_abad[ i ] * sig[ i * 3 + 0 ];
        robot_data_.qd[ i * 3 + 1 ]  = msg->qd_hip[ i ] * sig[ i * 3 + 1 ];
        robot_data_.qd[ i * 3 + 2 ]  = msg->qd_knee[ i ] * sig[ i * 3 + 2 ];
        robot_data_.tau[ i * 3 + 0 ] = msg->tau_abad[ i ] * sig[ i * 3 + 0 ];
        robot_data_.tau[ i * 3 + 1 ] = msg->tau_hip[ i ] * sig[ i * 3 + 1 ];
        robot_data_.tau[ i * 3 + 2 ] = msg->tau_knee[ i ] * sig[ i * 3 + 2 ];
    }
    for ( int i = 0; i < 12; i++ ) {
        robot_data_.motor_flags[ i ] = msg->flags[ i ];
    }
}
void CustomInterface::Handle_LegData_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const leg_control_data_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    // msg->p[ 12 ];
    // msg->v[ 12 ];
    // msg->force_est[ 12 ];
}
void CustomInterface::Handle_RobotState_Lcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    for ( int i = 0; i < 3; i++ ) {
        robot_data_.omega[ i ] = msg->omegaWorld[ i ];
        robot_data_.rpy[ i ]   = msg->rpy[ i ];
        robot_data_.acc[ i ]   = msg->aWorld[ i ];
    }
    for ( int i = 0; i < 4; i++ ) {
        robot_data_.quat[ i ] = msg->quat[ i ];
    }
}