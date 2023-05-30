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

class CustomCtrl : public CustomInterface {
public:
    CustomCtrl( const double& loop_rate ) : CustomInterface( loop_rate ){};
    ~CustomCtrl(){};

private:
    long long count_ = 0;
    float     init_q_[ 12 ];
    float     target1_q_[ 3 ] = { 0 / 57.3, 80 / 57.3, -135 / 57.3 };
    float     target2_q_[ 3 ] = { 0 / 57.3, 45 / 57.3, -70 / 57.3 };
    void      UserCode( bool first_run ) {
        //  vertical view
        // leg 1 | | leg 0
        //       | |
        // leg 3 | | leg 2
        //
        // Right hand coordinate system
        //   ______    zero angle   ______
        //   \     \     -->        |     |
        //   /     /                |     |

        float t = ( count_ / 1500.0 ) > 2 ? 2 : ( count_ / 1500.0 );
        if ( first_run == true ) {
            for ( int i = 0; i < 12; i++ )
                init_q_[ i ] = robot_data_.q[ i ];
            if ( init_q_[ 2 ] < -0.1 && init_q_[ 5 ] < -0.1 && init_q_[ 8 ] < -0.1 && init_q_[ 11 ] < -0.1 ) {
                count_ = -1;
            }
        }
        else {
            for ( int i = 0; i < 12; i++ ) {
                if ( t < 1.0 )
                    motor_cmd_.q_des[ i ] = target1_q_[ i % 3 ] * t + init_q_[ i ] * ( 1 - t );
                else
                    motor_cmd_.q_des[ i ] = target2_q_[ i % 3 ] * ( t - 1 ) + target1_q_[ i % 3 ] * ( 2 - t );
                motor_cmd_.kp_des[ i ]  = 60;
                motor_cmd_.kd_des[ i ]  = 2;
                motor_cmd_.qd_des[ i ]  = 0;
                motor_cmd_.tau_des[ i ] = 0;
            }
        }

        if ( ( count_++ ) % 1000 == 0 )
            PrintData( robot_data_ );
    }
};

std::shared_ptr< CustomCtrl > io;

void signal_callback_handler( int signum ) {
    io->Stop();
    ( void )signum;
}

int main() {
    /* user code ctrl mode:1 for motor ctrl */
    signal( SIGINT, signal_callback_handler );
    io = std::make_shared< CustomCtrl >( 500 );
    io->Spin();
    return 0;
}