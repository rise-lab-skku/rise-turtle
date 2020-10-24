
rosshutdown
rosinit()

node = robotics.ros.Node('my_node'); 
myPub = robotics.ros.Publisher(node, '/test', 'std_msgs/Int32', "IsLatching", true);
myMsg = rosmessage(myPub);

myMsg.Data = 123;
send(myPub, myMsg) % Publish only once
