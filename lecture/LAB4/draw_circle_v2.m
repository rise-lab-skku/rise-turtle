rosshutdown % Close the old session
rosinit()   % Initialize new session
[myPub, myMsg] = rospublisher('/turtle1/cmd_vel', 'geometry_msgs/Twist');
msg = rossubscriber('/turtle1/pose', 'turtlesim/Pose');

rate = robotics.Rate(1);    % 1 Hz
reset(rate);
for i = 1:60
    if isempty(msg.LatestMessage)==false
        disp(msg.LatestMessage);
    else
        disp("Pose : empty");
    end
    myMsg.Linear.X = 2.0;   % Change this dynamically
    myMsg.Angular.Z = 1.5;  % Change this dynamically
    send(myPub, myMsg)
    waitfor(rate);
end
