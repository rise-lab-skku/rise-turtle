rosshutdown % Close the old session
rosinit()   % Initialize new session
[myPub, myMsg] = rospublisher('/turtle1/cmd_vel', 'geometry_msgs/Twist');

rate = robotics.Rate(1);    % 1 Hz
reset(rate);
for i = 1:60
    myMsg.Linear.X = 2.0;   % Change this dynamically
    myMsg.Angular.Z = 1.5;  % Change this dynamically
    send(myPub, myMsg)
    waitfor(rate);
end
