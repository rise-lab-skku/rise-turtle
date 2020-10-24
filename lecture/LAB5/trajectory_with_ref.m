%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SKKU EME3071
% ------------
% This code is written in PP(Procedural Programming).
% If you change them to OOP(Object-oriented Programming),
% then it will be easier to read and modify and fix.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rosshutdown;            % Clean up
rosinit('192.168.4.1'); % Initialize the node

%=========== Parameters ===========
% http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#hardware-specifications
wheel_radius = 0.0325;       % [m]
wheel_interval = 0.158;      % 2L [m]
L = wheel_interval / 2.0;
max_linear_vel = 0.22;       % [m/sec]
max_rotational_vel = 2.84;   % [rad/sec]

%=========== Subscribers ===========
pose_sub = rossubscriber('/odom');
lidar_sub = rossubscriber('/scan');
pause(1);  % Ready for the first receiving

%=========== Publishers ===========
% The inside of '/wheel_control' topic:
%     X: Operation modes of dynamixel (1:velocity mode, 16:pwm mode)
%     Y: Left motor    [m/s] or (-1.0 ~ 1.0)
%     Z: Right motor   [m/s] or (-1.0 ~ 1.0)
vel_pub = rospublisher('/wheel_control', 'geometry_msgs/Vector3');
vel_msg = rosmessage(vel_pub);

% Operation modes
VELOCITY_MODE = 1;
PWM_MODE = 16;

%=========== For PID ===========
% X error
Kx_p = 0.5;
Kx_i = 0;
Kx_d = 0;

% Heading error (rP_theta)
Kt_p = 1.7;
Kt_i = 0;
Kt_d = 0;

%=========== Goal ===========
% Trajectory: X, Y, ref_linear_velocity
Traj = [  0.00,  0.05,  0.01;
          0.00,  0.10,  0.13;
          0.00,  0.15,  0.13;
          0.00,  0.20,  0.13;
          0.00,  0.25,  0.13;
          0.00,  0.30,  0.13;
          0.00,  0.35,  0.13;
          0.00,  0.40,  0.13;
          
          0.05,  0.35,  0.01;
          0.10,  0.30,  0.13;
          0.15,  0.25,  0.13;
          0.20,  0.20,  0.13;
          
          0.15,  0.16,  0.01;
          0.10,  0.12,  0.13;
          0.05,  0.08,  0.13;
          0.00,  0.04,  0.13;
         -0.05,  0.00,  0.13;
         
          0.00,  0.00,  0.00 ];
      
[row, column] = size(Traj);  
LastPoint = row;

% Target point "P" in the coordinate system "I"
% Unit: [m]
index = 1;
iP = [ Traj(index, 1); Traj(index, 2) ];

% Circular mask for termination condition
% Unit: [m]
Look_Ahead = 0.02;  % 2cm

%=========== Loop ===========
rate = rosrate(20);  % 20Hz
reset(rate);
initialTime = rostime("now");
while true
    % Read the data by subscriber
    %=========== topic: '/odom' ===========
    position = pose_sub.LatestMessage.Pose.Pose.Position;
    quat = pose_sub.LatestMessage.Pose.Pose.Orientation;
    euler_angle = quat2eul([quat.X quat.Y quat.Z quat.W]);
    
    % Current pose of the robot "Q" in the coordinate system "I"
    %     [Comment]: Using "R" as the robot symbol can be confusing
    %                with the rotation matrix. So we use Q here.
    iQ = [ position.X; position.Y ];
    Heading = euler_angle(3);  % Current Heading [radian]
    
    %=========== Mobile Robot Kinematics ===========
    % This section is from Slide.31 of the lecture note "LAB5".
    
    % Transform from "I" coordinate to "R" coodinate
    iD = iP - iQ;
    
    ri_R = [  cos(Heading), sin(Heading);        
             -sin(Heading), cos(Heading) ];   % Rotation matrix "R" to "I"
         
    rP = ri_R * iD;                           % We need rP!
    
    rP_theta = atan2(rP(2), rP(1));           % arctan2(Y,X)
    
    %=========== PID Controller ===========
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This is NOT the Kanayama Control (Slide.43).
    % But similar idea.
    % If you can understand this method, you can implement the Kanayama.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Reference velocity
    ref_linear  = Traj(index, 3) * cos(rP_theta);
        
    % Example
    linear_velocity  = Kx_p * rP(1)     + ref_linear;
    angular_velocity = Kt_p * rP_theta;
    
    % Constraints
    linear_velocity = min(linear_velocity, max_linear_vel);
    linear_velocity = max(linear_velocity, -max_linear_vel);
    %angular_velocity = min(angular_velocity, max_rotational_vel);
    %angular_velocity = max(angular_velocity, -max_rotational_vel);

    %=========== Velocity -> Commend ===========
    left_wheel  = linear_velocity - (L * angular_velocity);
    right_wheel = linear_velocity + (L * angular_velocity);
    
    %=========== Publishing ===========
    vel_msg.X = VELOCITY_MODE;
    vel_msg.Y = left_wheel;
    vel_msg.Z = right_wheel;
    send(vel_pub, vel_msg);
    
    %=========== Termination condition ===========
    distance = sqrt( rP(1)*rP(1) + rP(2)*rP(2) ); % Using 'square root'
    
    if distance < Look_Ahead
        index = index + 1;
    end
    
    if index > LastPoint
        vel_msg.Y = 0;
        vel_msg.Z = 0;
        send(vel_pub, vel_msg);
        break
    else
        iP = [ Traj(index, 1); Traj(index, 2) ];
    end
    
    %=========== Debugging ===========
    % Just printing values
    % seconds(rostime("now")-initialTime) is Current Time
    [ seconds(rostime("now")-initialTime), distance, 0, 0;
      linear_velocity, angular_velocity, left_wheel, right_wheel ]
    
    waitfor(rate);
end