node = robotics.ros.Node('my_listener', 'localhost', 11311);
msg = robotics.ros.Subscriber(node, '/my_first_topic', 'std_msgs/Int32');

rate = robotics.ros.Rate(node, 2);  % 2 Hz
reset(rate);
while true
    if isempty(msg.LatestMessage)==false
        disp("Listener : " + msg.LatestMessage.Data(1));
    else
        disp("Listener : empty");
    end
    waitfor(rate);
end
