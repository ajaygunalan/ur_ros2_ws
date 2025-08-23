addpath(genpath(fullfile('..', '..', '..', '..', '..', '..', 'simulations', 'pa_point_sources', 'images')));
clear all

% Shut down the previous instance of this ROS node, if required.
rosshutdown;
rosinit;
ros_cleanup = onCleanup(@rosshutdown);
% ros_timer = timer();
% set(ros_timer, 'executionMode', 'fixedRate');
% set(ros_timer, 'TimerFcn', 'imgPublishFunction()');
% set(ros_timer, 'Period', 1.0);
% start(ros_timer);

while(1)
	fastImgPublishFunction();
	pause(1.0);
end

% stop(ros_timer);
