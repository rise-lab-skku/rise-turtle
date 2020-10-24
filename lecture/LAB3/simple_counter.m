node = robotics.ros.Node('my_talker', 'localhost', 11311);
 
pub = robotics.ros.Publisher(node, '/my_first_topic','std_msgs/Int32');
msg = rosmessage(pub);
 
rate = robotics.ros.Rate(node, 1);  % 1 Hz
reset(rate);
 
msg.Data = 0;
while true
    send(pub,msg);
    msg.Data = msg.Data + 1;
    waitfor(rate);
end
