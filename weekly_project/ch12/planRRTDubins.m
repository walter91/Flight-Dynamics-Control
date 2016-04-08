% planRRTDubins.m

% how handle connecting to end node?
%   - currently test if segmentLength <= distance <= 2*segmentLength
% how handle segments cannot be less than 2*R_min?
%   - currently fix distance between nodes at segmentLength
% how handle collision checking?
%   - currently checks straight line portion of dubins path
% how draw dubins tree?

function path_out = planRRTDubins(wpp_start, wpp_end, R_min, map)

    % standard length of path segments
    segmentLength = 2.2*R_min;

    % set fixed height for path
    pd = wpp_end(3);

    % define start and end nodes
    start_node = [wpp_start(1), wpp_start(2), pd, wpp_start(4), 0, 0, 0];
    end_node = [wpp_end(1), wpp_end(2), pd, wpp_end(4), 0, 0, 0];
    % format:  [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
    % establish tree starting with the start node
    tree = start_node;
    
    % check to see if start_node connects directly to end_node
    dist = norm(start_node(1:3)-end_node(1:3));
    endpath = dubinsParameters(start_node,end_node,R_min);
    if isempty(endpath),
        disp('No valid Dubins path');
    elseif (dist>=(2*R_min)) && (dist<(2*segmentLength)) ...
        && (collision(endpath,pd,map)==0),
%         path = [start_node; end_node];
        tree(end,7) = 1;
    else
        numPaths = 0;
        while numPaths<3,
            [tree,flag] = extendTree(tree,end_node,segmentLength,map,pd,R_min);
            numPaths = numPaths + flag;
        end
    end
    
    % find path with minimum cost to end_node
    path = findMinimumPath(tree,end_node);
%     path_out = path;
    path_out = smoothPath(path,map,R_min,pd);
    plotmap(map,path,path_out,tree,R_min,pd);

end


%% Add New Node to Tree
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,map,pd,R_min)

  flag1 = 0;
  while flag1==0,
    % select a random point
    randomNode=generateRandomNode(map,pd);
    
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:3)-ones(size(tree,1),1)*randomNode(1:3);
    [~,idx] = min(diag(tmp*tmp'));
    tmp = randomNode(1:3)-tree(idx,1:3);
    new_point = tree(idx,1:3)+segmentLength*(tmp/norm(tmp));
    
    chi = -(atan2(tmp(1),tmp(2)) - pi/2); % set heading as angle of line that connects new_point to tree
    newpath = dubinsParameters(tree(idx,:), [new_point,chi,0,0], R_min);
    if ~isempty(newpath),
        cost = tree(idx,5) + newpath.L;

        new_node = [new_point, chi, cost, idx, 0];
        if collision(newpath, pd, map)==0,
          new_tree = [tree; new_node];
          flag1=1;
        end
    end
  end
  
  % check to see if new node connects directly to end_node
  dist = norm(new_node(1:3)-end_node(1:3));
  if (dist>=(2*R_min)) && (dist<(2*segmentLength)),
      endpath = dubinsParameters(new_node,end_node,R_min);
      if ~isempty(endpath),
          if collision(endpath, pd, map)==0,
              flag = 1;
              new_tree(end,7) = 1; % mark node as connecting to end.
          else
              flag = 0;
          end
      else
          flag = 0;
      end
  else
    flag = 0;
  end
  
end


% Generate Random Node
function node=generateRandomNode(map,pd)

    % randomly pick configuration
    pn       = map.width*rand;
    pe       = map.width*rand;
    pd       = pd; % constant altitute paths
    cost     = 0;
    node     = [pn, pe, pd, 0, cost, 0, 0];
    % format:  [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
end


%% Collision Check
function collision_flag = collision(dubinspath, pd, map)

    collision_flag = 0;
    
    [X,Y] = pointsAlongDubinsPath(dubinspath,0.1);

    for i = 1:length(X),
        if pd >= downAtNE(map, X(i), Y(i)),
            collision_flag = 1;
        end
    end
    
end

% Find Points Along Dubins Path
function [X,Y] = pointsAlongDubinsPath(dubinspath,Del)


  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  X = [];
  Y = [];
  for i=1:length(th),
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1,
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  for i=1:length(th),
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
  end
  
end

% Find Down Value At Given Point
function down = downAtNE(map, n, e)

      [d_n,idx_n] = min(abs(n - map.buildings_n));
      [d_e,idx_e] = min(abs(e - map.buildings_e));

      if (d_n<=map.BuildingWidth) && (d_e<=map.BuildingWidth),
          down = -map.heights(idx_e,idx_n);
      else
          down = 0;
      end

end


%% Generate Path
% Find Minimum Path
function path = findMinimumPath(tree,end_node)
    
    % find nodes that connect to end_node
    connectingNodes = [];
    for i=1:size(tree,1),
        if tree(i,7)==1,
            connectingNodes = [connectingNodes; tree(i,:)];
        end
    end

    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,5));

    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,6);
    while parent_node>1,
        parent_node = tree(parent_node,6);
        path = [tree(parent_node,:); path];
    end
    
end

% Smooth Path
function newPath = smoothPath(path,map,R_min,pd)

    newPath = path(1,:); % add the start node 
    ptr =2;  % pointer into the path
    while ptr <= size(path,1)-1,
        dubinspath = dubinsParameters(newPath(end,:),path(ptr+1,:),R_min);
        if isempty(dubinspath) || (collision(dubinspath, pd, map)~=0), % if there is a collision
            newPath = [newPath; path(ptr,:)];  % add previous node
        end
        ptr=ptr+1;
    end
    newPath = [newPath; path(end,:)];

end

%% Generate Plot
% Plot Obstacles and Path
function plotmap(map,path,smoothedPath,tree,R_min,pd)
  
    % setup plot
    figure(3), clf
    axis([-500,map.width+500,-500,map.width+500,0,2*map.MaxHeight]);
    xlabel('E')
    ylabel('N')
    zlabel('h')
    view([0,90])
    hold on
  
    % plot buildings 
    V = [];
    F = [];
    patchcolors = [];
    count = 0;
    for i=1:map.NumBlocks,
        for j=1:map.NumBlocks,
            [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
                map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
            V = [V; Vtemp];
            Ftemp = Ftemp + count;
            F = [F; Ftemp];
            count = count + 8;
            patchcolors = [patchcolors;patchcolorstemp];
        end
    end
  
    patch('Vertices', V, 'Faces', F,...
                     'FaceVertexCData',patchcolors,...
                     'FaceColor','flat');
    
    % draw tree
    for i=2:size(tree,1),
        dpath = dubinsParameters(tree(i,:), tree(tree(i,6),:), R_min);
        [X,Y] = pointsAlongDubinsPath(dpath,0.1);
        Z = pd*ones(1,size(X,1));
        plot3(Y,X,-Z,'g');
    end

    % draw path
    for i=1:(size(path,1)-1),
        dpath = dubinsParameters(path(i,:), path((i+1),:), R_min);
        [X,Y] = pointsAlongDubinsPath(dpath,0.1);
        Z = pd*ones(1,size(X,1));
        plot3(Y,X,-Z,'r','linewidth',2);
    end

    % draw smooth path
    for i=1:(size(smoothedPath,1)-1),
        dpath = dubinsParameters(smoothedPath(i,:), smoothedPath((i+1),:), R_min);
        [X,Y] = pointsAlongDubinsPath(dpath,0.1);
        Z = pd*ones(1,size(X,1));
        plot3(Y,X,-Z,'k','linewidth',2);
    end

end

% Define Buildings
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end

    