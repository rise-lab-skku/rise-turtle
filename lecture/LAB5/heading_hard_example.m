rosshutdown;            % Clean up
rosinit('192.168.4.1'); % Initialize the node

%=========== Subscribers ===========
pose_sub = rossubscriber('/odom');
lidar_sub = rossubscriber('/scan');
pause(1);  % Ready for the first receiving

%=========== Publishers ===========
% The inside of '/wheel_control' topic
% X: Operation modes of dynamixel (1:velocity mode, 16:pwm mode)
% Y: Left motor  (m/s or -1.0~1.0)
% Z: Right motor (m/s or -1.0~1.0)
vel_pub = rospublisher('/wheel_control', 'geometry_msgs/Vector3');
vel_msg = rosmessage(vel_pub);

%=========== Settings ===========
target_Heading = pi/2.0;  % 90 deg
%Kp = 0.5;  % For velocity mode
Kp = 5;  % For PWM mode
Ki = 0;
Kd = 0;

not_initialized = true;

%=========== Parameters ===========
wheel_radius = 0.0325;       % [m]
wheel_interval = 0.158;      % 2L [m]
L = wheel_interval / 2.0;
max_linear_vel = 0.22;       % [m/sec]
max_rotational_vel = 2.84;   % [rad/sec]

%=========== Placeholders ===========
integral = 0;    % For I-control
prev_Error = 0;  % For D-control

%=========== Loop ===========
rate = rosrate(20);  % 20Hz
reset(rate);
while true
    % Read the data by subscriber
    %=========== topic: '/odom' ===========
    position = pose_sub.LatestMessage.Pose.Pose.Position;
    quat = pose_sub.LatestMessage.Pose.Pose.Orientation;  % Quaternion
    euler_angle = quat2eul([quat.X quat.Y quat.Z quat.W]);
    
    X = position.X;            % Current X [m]
    Y = position.Y;            % Current Y [m]
    Heading = euler_angle(3);  % Current Heading [Radian]
    
    %=========== Initialize values ===========
    if not_initialized
        target_Heading = target_Heading + Heading;
        not_initialized = false;
    end
    
    %=========== Calculate error ===========
    error_Heading = target_Heading - Heading;
    
    %=========== Normalize error ===========
    % Set the range of error_Heading to (-pi ~ pi)
    if (error_Heading > pi)
        error_Heading = error_Heading - 2*pi;
    elseif (error_Heading < -pi)
        error_Heading = error_Heading + 2*pi;
    end
    
    %=========== PID Controller ===========
    linear_cmd = 0;
    angular_cmd = Kp * error_Heading;
    
    %=========== Velocity -> Commend ===========
    operation_mode = 1;   % Velocity mode
    %operation_mode = 16;   % PWM mode
    left_vel  = linear_cmd - (L * angular_cmd);
    right_vel = linear_cmd + (L * angular_cmd);
    
    vel_msg.X = operation_mode;
    vel_msg.Y = left_vel;
    vel_msg.Z = right_vel;
    
    %=========== Publishing ===========
    send(vel_pub, vel_msg);
    
    waitfor(rate);
end