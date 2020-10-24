
1. Connect your WiFi to "Turtle#_skkurise"(**DO NOT USE extra network adapter!**)
2. In linux, open the "terminal" and write "ifconfig"
3. When your **TURTLE_IP** is 192.168.4.1, your **REMOTE_IP** is 192.168.4.xx
4. In linux, open the "terminal" and write "gedit ~/.bashrc"
5. In the gedit, change "export ROS_HOSTNAME=192.168.x.x" to "export ROS_HOSTNAME=**REMOTE_IP**" and save it
6. In linux, open the new "terminal" and write "ssh pi@**TURTLE_IP**" (For example, **TURTLE_IP** is 192.168.4.1)
7.  The password is "rise"
8.  In your turtle, try "ping **REMOTE_IP**" to check your connection!
9.  In your turtle, "roslaunch turtlebot3_bringup turtlebot3_robot.launch"


# Subscriber

| Topic Name | Type | Data | Origin? |
| ----- | ----- | ----- | ----- |
| wheel_control | geometry_msgs::Vector3 | .x: motor mode (1:vel, 16:pwm) <br/>.y: left [m/s, -1.0~1.0] <br/>.z: right [m/s, -1.0~1.0] | no |
| left_pwm | std_msgs::Float32 | .data: [m/s, -1.0~1.0] | no |
| right_pwm | std_msgs::Float32 | .data: [m/s, -1.0~1.0] | no |
| cmd_vel | geometry_msgs::Twist | .linear.x: [m/s] <br/>.angular.z: [rad] | yes |
| sound | turtlebot3_msgs::Sound | .value: unsigend int<br/>(0:off, 1:on, 2:low_battery, 3:error, 4~5:no sound, 6~:on_2nd) | yes |
| motor_power | std_msgs::Bool | .data: True/False (Set torque) | yes |
| reset | std_msgs::Empty | (empty): {}<br/>(Start Calibration of Gyro, Reset Odometry) | yes |


# Publisher

| Topic Name | Hz | Type | Data | Origin? |
| ----- | ----- | ----- | ----- | ----- |
| tf | 30 | - | - | yes |
| sensor_state | 30 | turtlebot3_msgs::SensorState | .header.stamp = ros::Time<br/>.battery = voltage [v]<br/>.bumper = disabled<br/>.button = disabled<br/>.cliff = disabled<br/>.illumination = disabled<br/>.led = not implemented<br/>.left_encoder = tick (1 tick is 0.087890625[deg])<br/>.right_encoder = tick (1 tick is 0.087890625[deg])<br/>.sonar = not implemented<br/>.torque = True(enable), False(disable)<br/>**ENCODER:**<br/>* Operating Mode(11)가 위치 제어 모드로 변경되는 시점에 1 [rev] (0 ~ 4,095) 범위로 초기화<br/>* 위치 제어 모드에서 Torque ON으로 변경되는 시점에 1 [rev] (0 ~ 4,095) 범위로 초기화 | yes |
| imu | 200 | sensor_msgs::Imu | .angular_velocity: [rad]<br/>.linear_acceleration: [m/sec2]<br/>.orientation: quaternion | yes |
| odom | 30 | nav_msgs::Odometry | .pose.pose.position.x = odom_pose[0]<br/>.pose.pose.position.y = odom_pose[1]<br/>.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2])<br/>.twist.twist.linear.x = odom_vel[0]<br/>.twist.twist.linear.z = odom_vel[2] | yes |
| joint_states | 30 | sensor_msgs::JointState | Joint(Dynamixel) state<br/>.position: (left [rad], right [rad])<br/>.velocity: (d_tick->d_rad)->d_rad/d_millisec (left [rad/sec], right [rad/sec]) | yes |
| magnetic_field | 200 | sensor_msgs::MagneticField | Magnetometer 3Axis (MPU9250) | yes |
| opmode_state | 30 | std_msgs::UInt8 | .data: operating mode(1:vel, 16:pwm) | no |
| present_pwm_L_state | 30 | std_msgs::Int16 | .data: (-885 ~ 885) | no |
| present_pwm_R_state | 30 | std_msgs::Int16 | .data: (-885 ~ 885) | no |
| present_vel_L_state | 30 | std_msgs::Int32 | .data: ([0.229rev/min]) | no |
| present_vel_R_state | 30 | std_msgs::Int32 | .data: ([0.229rev/min]) | no |
| diagnostics | 1 | maybe in the raspberry pi | - | yes |
| rpms | 5 | maybe in the raspberry pi | [revolution/min] | yes |
| scan | 5 | maybe in the raspberry pi | .ranges, .range_min, .range_max: [m] | yes |


# Publisher

| Topic Name | Hz | Type | Data | Origin? |
| ----- | ----- | ----- | ----- | ----- |
| tf | 30 | - | - | yes |
| sensor_state | 30 | turtlebot3_msgs::SensorState | .header.stamp = ros::Time<br/>.battery = voltage [v]<br/>.bumper = disabled<br/>.button = disabled<br/>.cliff = disabled<br/>.illumination = disabled<br/>.led = not implemented<br/>.left_encoder = tick (1 tick is 0.087890625[deg])<br/>.right_encoder = tick (1 tick is 0.087890625[deg])<br/>.sonar = not implemented<br/>.torque = True(enable), False(disable)<br/>**ENCODER:**<br/>* Operating Mode(11)가 위치 제어 모드로 변경되는 시점에 1 [rev] (0 ~ 4,095) 범위로 초기화<br/>* 위치 제어 모드에서 Torque ON으로 변경되는 시점에 1 [rev] (0 ~ 4,095) 범위로 초기화 | yes |
| firmware_version | 1 | turtlebot3_msgs::VersionInfo | disabled | yes |
| imu | 200 | sensor_msgs::Imu | .angular_velocity: [rad]<br/>.linear_acceleration: [m/sec2]<br/>.orientation: quaternion | yes |
| odom | 30 | nav_msgs::Odometry | .pose.pose.position.x = odom_pose[0]<br/>.pose.pose.position.y = odom_pose[1]<br/>.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2])<br/>.twist.twist.linear.x = odom_vel[0]<br/>.twist.twist.linear.z = odom_vel[2] | yes |
| joint_states | 30 | sensor_msgs::JointState | Joint(Dynamixel) state<br/>.position: (left [rad], right [rad])<br/>.velocity: (d_tick->d_rad)->d_rad/d_millisec (left [rad/sec], right [rad/sec]) | yes |
| battery_state | 30 | sensor_msgs::BatteryState | disabled | yes |
| magnetic_field | 200 | sensor_msgs::MagneticField | Magnetometer 3Axis (MPU9250) | yes |
| opmode_state | 30 | std_msgs::UInt8 | .data: operating mode(1:vel, 16:pwm) | no |
| torque_state | 30 | std_msgs::Bool | disabled | no |
| hw_error_L_state | 30 | std_msgs::UInt8 | disabled | no |
| hw_error_R_state | 30 | std_msgs::UInt8 | disabled | no |
| goal_pwm_L_state | 30 | std_msgs::Float32 | disabled | no |
| goal_pwm_R_state | 30 | std_msgs::Float32 | disabled | no |
| goal_vel_L_state | 30 | std_msgs::Float32 | disabled | no |
| goal_vel_R_state | 30 | std_msgs::Float32 | disabled | no |
| present_pwm_L_state | 30 | std_msgs::Int16 | .data: (-885 ~ 885) | no |
| present_pwm_R_state | 30 | std_msgs::Int16 | .data: (-885 ~ 885) | no |
| present_vel_L_state | 30 | std_msgs::Int32 | .data: ([0.229rev/min]) | no |
| present_vel_R_state | 30 | std_msgs::Int32 | .data: ([0.229rev/min]) | no |
| diagnostics | 1 | maybe in the raspberry pi | - | yes |
| rpms | 5 | maybe in the raspberry pi | [revolution/min] | yes |
| scan | 5 | maybe in the raspberry pi | .ranges, .range_min, .range_max: [m] | yes |