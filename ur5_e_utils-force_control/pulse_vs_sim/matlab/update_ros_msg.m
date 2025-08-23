vision_msg_path = fullfile('..', '..', 'matlab_msg_gen_ros1', 'glnxa64', 'install', 'm');
rosgenmsg(fullfile('..', '..'));
addpath(vision_msg_path);
clear classes
rehash toolboxcache
