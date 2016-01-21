
function drawAircraft(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        aircraft_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-20,20,-20,20,-20,20]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           aircraft_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = translate(V, pn, pe, pd);  % translate vehicle
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

fuse_l1 = 7;
fuse_l2 = 4;
wing_l = 6;
fuse_l3 = 15;
tail_h = 3;
fuse_h = 3;
fuse_w = 2;
wing_w = 20;
tailwing_w = 10;
tailwing_l = 3;

% Define the vertices (physical location of vertices
V = [fuse_l1,	0,	0;...
    fuse_l2,	fuse_w/2,	-fuse_h/2;...
    fuse_l2,	-fuse_w/2,	-fuse_h/2;...
    fuse_l2,	-fuse_w/2,	fuse_h/2;...
    fuse_l2,	fuse_w/2,	fuse_h/2;...
    -fuse_l3,	0	0;...
    0,	wing_w/2,	0;...
    -wing_l,	wing_w/2,	0;...
    -wing_l,	-wing_w/2,	0;...
    0,	-wing_w/2,	0;...
    -fuse_l3+tailwing_l,	tailwing_w/2,	0;...
    -fuse_l3,	tailwing_w/2,	0;...
    -fuse_l3,	-tailwing_w/2,	0;...
    -fuse_l3+tailwing_l,	-tailwing_w/2,	0;...
    -fuse_l3+tailwing_l,	0,	0;...
    -fuse_l3,	0,	-tail_h]';

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 3;...  % 
        1, 3, 4;...  % 
        1, 4, 5;...  % 
        1,	5,	2;...
        2,	6,	3;...
        3,	6,	4;...
        4,	6,	5;...
        5,	6,	2;...
        15,	16,	6;...
        11,	12,	13;...
        13, 14, 11;...
        7,	8,	9;...
        9,	10, 7];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    myblue;...     % tail
    ];
end

