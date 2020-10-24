rosshutdown;            % Clean up
rosinit('192.168.4.1'); % Initialize the node

%=========== Publishers ===========
vel_pub = rospublisher('/wheel_control', 'geometry_msgs/Vector3');
vel_msg = rosmessage(vel_pub);

%=========== Velocity -> Commend ===========
VELOCITY_MODE = 1;
PWM_MODE = 16;

vel_msg.X = PWM_MODE;
vel_msg.Y = 0;
vel_msg.Z = 0;

%=========== Publishing ===========
send(vel_pub, vel_msg);