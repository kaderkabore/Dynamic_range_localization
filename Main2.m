%--------------------------------------------------------------------------
%  Kader M KABORE
%  
%  Paper Simulation
%
%  TITLE : Dynamic Range-Only Localization for Multi-Robot Systems
%          https://ieeexplore.ieee.org/document/8443328
%--------------------------------------------------------------------------

clear
clc
close all

%% Initialization
%
%   Define the agents positions
%   Here we define 6 agents

st = 0.1;
t  =  10;
time = st:st:t;
ub = 5;                        % Upper bound for random initial condition
lb = -5;                       % Lower bound for random initial condition
x_coor = 1;
y_coor = 0;


SYSTEMSIZE = 15;                % Number Of agents
SENSOR_RANGE = 3;              % The range of the Uwb
THRESHOLD = 2.5;                 % Threshold for reconfiguration request

n = SYSTEMSIZE;
for i=1 : n
   pos(:,i,1) = [x_coor y_coor]'+(lb+(ub-lb)*rand(2,1));

end

%% Leader Selection
% Selects agent with the most neighbors
neighbors = zeros(n,1);

for i=1 : n
    for j=1 :n 
        if (i ~=j )
            dis = norm(pos(:,i,1)-pos(:,j,1));
            if (dis < SENSOR_RANGE )
                neighbors(i) =  neighbors(i) + 1;
                neighbors_ids(i, neighbors(i)) = j;
    
            end

        end


    end
end

% Selecting the first leader with higest bid
[count,ind] = max(neighbors);

%% Coordinate System Initialization

% Check Inline function
refs_count =  1;
for i=1:1:neighbors_ids(ind)
    for j=1:1:neighbors_ids(ind)
         if (i ~=j )
             % Check Line function
            isLine = Check_Inline(pos(:,ind),pos(:,i),pos(:,j)); 
             % check the distance to references
            dis1 = norm(pos(:,ind)-pos(:,i));   
            dis2 = norm(pos(:,ind)-pos(:,j)); 
            dis  = dis1 + dis2;

            if (isLine == 0)
                referencesIds(:,refs_count) = [ind,i,j];
                distances_toLeadder (refs_count) = dis ;
                refs_count = refs_count + 1;

            end
         end

    end

end
% Select the closest Not Inline to leader
[dis_ll,index_ll] = min(distances_toLeadder);

% The references robots indices
 leader_index = referencesIds(1,index_ll);
 ref1_index = referencesIds(2,index_ll);
 ref2_index = referencesIds(3,index_ll);
% The references robots Positions
 leader = pos(:,leader_index);
 ref1 = pos(:,ref1_index);
 ref2 = pos(:,ref2_index);


 %% The Building the common coordinate system

 z_la = norm(leader-ref1) + randn(1);
 z_lb = norm(leader-ref2) + randn(1);
 z_ab = norm(leader-ref2) + randn(1);

 xa = 1/2 * ( z_ab + ((z_la)^2 - (z_lb)^2)/z_ab);
 xb = 1/2 * ( - z_ab + ((z_la)^2 - (z_lb)^2)/z_ab);
 yl = sqrt(z_la - xa^2 );
 pos1 = [0, yl];
 pos2 = [xa, 0];
 pos3 = [xb, 0];


 %% Trilaterating 
 % Trilateration to find position To the common coordinate system
 for i=1:n
     % Leader
     if (i == leader_index )
         new_pos(:,i) = pos1;
     % ref1
     elseif (i == ref1_index )
          new_pos(:,i) = pos2;
     % ref2
     elseif (i == ref2_index )
          new_pos(:,i) = pos2;

     else
         dis1 = norm(pos(:,i)- leader) ;
         dis2 = norm(pos(:,i)- ref1) ;
         dis3 = norm(pos(:,i)- ref2) ;
         [x, y] = trilateration(pos1,dis1,pos2,dis2,pos2,dis3);
         new_pos(:,i) = [x, y];
         

     end

 end

 %% RECONFIGURATION 

 % Find Marginal Robots
 m1 = new_pos(:,1);
 m2 = new_pos(:,2);
 m3 = new_pos(:,3);
 m4 = new_pos(:,4);

 for i=1:n
     % Loop through all new points
     
     % Check y axis +
     if ( new_pos(2,i) > m1)
         m1 = new_pos(:,i);
     end
     % Check y axis -
     if ( new_pos(2,i) < m3)
         m3 = new_pos(:,i);
     end
     % Check x axis +
     if ( new_pos(1,i) < m2)
         m3 = new_pos(:,i);
     end
     % Check x axis -
     if ( new_pos(1,i) < m4)
         m4 = new_pos(:,i);
     end


 end
 % Find DC point to the middle
 mx = (m1(1) + m3(1))/2;
 my = (m2(1) + m4(1))/2;
 Dc = [mx,my];

 % Checking for reconfiguration Request
 dis_cc = 0;
 dis_dc = 0;
 for i=1:n
     dis_cc = dis_cc +  norm(new_pos(:,i)- new_pos(:,leader_index));
     dis_dc = dis_dc +  norm(new_pos(:,i)- Dc);
 end

 % Find mean distances 
 mean_cc =  dis_cc/n;
 mean_dc =  dis_dc/n;
 if (abs(mean_cc - mean_dc) < THRESHOLD)
     %%%% ------ TRIGGER FOR RECONFIGURATION  ----  %%%%
     

 end




















