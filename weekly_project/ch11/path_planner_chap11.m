function out = path_planner_chap11(u,P)

numwaypoints = 4;
waypoints = zeros(1,5*P.size_waypoint_array);

% define waypoints
waypoints(1:5*1)  = [P.pn0, P.pe0, P.pd0, -9999, 35];
waypoints(6:5*2)  = [1500, 0, -133, 0, 35];
waypoints(11:5*3) = [0, 1500, -166, 0, 35];
waypoints(16:5*4) = [1500, 1500, -200, -pi/2, 35];

% dubin's waypoints
% waypoints(1:5*1)  = [P.pn0, P.pe0, P.pd0, 0, 35];
% waypoints(6:5*2)  = [1500, 1500, -100, 0*pi/180, 35];
% waypoints(11:5*3) = [1500, 0, -100, 180*pi/180, 35];
% waypoints(16:5*4) = [0, 1500, -100, 180*pi/180, 35];
% waypoints(21:5*5) = [P.pn0, P.pe0, P.pd0, 0, 35];

out = [numwaypoints waypoints]';

end