rosshutdown;                  % Clear the old section
rosinit('localhost', 11311);  % Start a new section

% Publisher
system('matlab -nosplash -nodesktop -r simple_counter -logfile output_counter.log &');
 
% Subscriber
system('matlab -nosplash -nodesktop -r simple_listener -logfile output_listener.log &');
