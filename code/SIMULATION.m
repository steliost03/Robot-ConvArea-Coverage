%{

Copyright (C) 2017 Stylianos Tsiakalos

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

%}

close all;
clear all;
clc;
warning off;

% environment %
xb = [0 2.125 2.9325 2.975 2.9325 2.295 0.85 0.17];
yb = [0 0 1.5 1.6 1.7 2.1 2.3 1.2];
patch(xb,yb,'white','edgecolor','blue');
hold on;

% robot data %

robotCount = 4;
%initial positions
px = [0.25 0.5 0.75 1];
py = [0.25 0.5 0.75 1];
plot(px,py,'k.');
axis([0 3.5 0 3.5])
hold on;

%sensing circle
R = 1;
%communication radius
Rc = 2;
%gain factor (for control law)
K = 5;
%fundamental time unit (the amount of time each iteration represents)
dt = 0.05;
%iteration limit
iterlim = 1000;
%convergence threshold
thr = 0.05*R;

iteration_count = 0;

%plot initial robot positions in first figure
title('INITIAL STATE');
for i = 1:robotCount
    [Xsense_init,Ysense_init] = approx_circle(R,[px(i);py(i)]);
    [Xcomms_init,Ycomms_init] = approx_circle(Rc,[px(i);py(i)]);
    plot(Xsense_init,Ysense_init,'k');
    plot(Xcomms_init,Ycomms_init,'--r');
    legend('Environment','Initial Positions','Sensing Area','Communication Area');
    hold on;
end

%initialization
trajectories_X = zeros(robotCount,iterlim); %each row for each robot
trajectories_Y = zeros(robotCount,iterlim);
for i=1:robotCount
    trajectories_X(i,1) = px(i);
    trajectories_Y(i,1) = py(i);
end

effective_area_progression = zeros(1,iterlim);


% MAIN SIMULATION LOOP %

for t=1:iterlim
    
    areas = zeros(1,robotCount);
    Xvoronoi = cell(1,robotCount);
    Yvoronoi = cell(1,robotCount);
    
    
    for j=1:robotCount
       
        %calculate current voronoi cell for every robot
        targetRobotPos = [trajectories_X(j,t) ; trajectories_Y(j,t)];
        allRobotPos = [trajectories_X(:,t)' ; trajectories_Y(:,t)'];
        [Xvoronoi{1,j},Yvoronoi{1,j}] = BoundedVoronoi(targetRobotPos,j,allRobotPos,xb,yb,Rc);
   
        
        %determine sensing circle positions
        current_center = [trajectories_X(j,t) ;trajectories_Y(j,t)];
        [Xcircles(:,j),Ycircles(:,j)] = approx_circle(R,current_center);
    end
    
  
    %calculate effective area
        %for every robot,find intersection of sensing circle with voronoi
        area = 0;
        for i = 1:robotCount
            [Xarea,Yarea] = polybool('intersection',Xcircles(:,i),Ycircles(:,i),Xvoronoi{1,i},Yvoronoi{1,i});
            area = area + polyarea(Xarea,Yarea);
        end
    
    effective_area_progression(t) = area;
    
    %find new position of each robot according to control law
    if(t+1 <= iterlim)
        termination_condition = 1;
        for j=1:robotCount
            targetRobotPos = [trajectories_X(j,t);trajectories_Y(j,t)];
            newTargetRobotPos = Control_Law(targetRobotPos,Xvoronoi{1,j},Yvoronoi{1,j},Xcircles(:,j),Ycircles(:,j),K,dt);
            diff_x = abs(newTargetRobotPos(1) - targetRobotPos(1));
            diff_y = abs(newTargetRobotPos(2) - targetRobotPos(2));
            diff = sqrt(diff_x*diff_x+diff_y+diff_y);
            if(diff >thr)
                termination_condition = 0;
            end
            trajectories_X(j,t+1) = newTargetRobotPos(1);
            trajectories_Y(j,t+1) = newTargetRobotPos(2);
        end
    end
    
    if(termination_condition == 1)
        iteration_count = t;
        break;
    end
      
end

if(termination_condition == 0)
    iteration_count = iterlim;
    trajectories_X_output = zeros(robotCount,iteration_count);
    trajectories_Y_output = zeros(robotCount,iteration_count);
    for i=1:robotCount
        trajectories_X_output(i,:) = trajectories_X(i,:);
        trajectories_Y_output(i,:) = trajectories_Y(i,:);
    end
    effective_area_progression_output = effective_area_progression;
end

if(termination_condition == 1)
    trajectories_X_output = zeros(robotCount,iteration_count);
    trajectories_Y_output = zeros(robotCount,iteration_count);
    effective_area_progression_output = zeros(1,iteration_count);
    for i=1:iteration_count
        effective_area_progression_output(i) = effective_area_progression(i);
        for j=1:robotCount
            trajectories_X_output(j,i) = trajectories_X(j,i);
            trajectories_Y_output(j,i) = trajectories_Y(j,i);
        end
    end
end

trajectories = [trajectories_X ; trajectories_Y];

%plot trajectories in second figure
colors = ['y';'m';'c';'r';'g';'b'];
figure;
patch(xb,yb,'white','edgecolor','blue');
hold on;
title('Trajectory progression ( * = final position )');
j = 1;
for i=1:robotCount
    plot(trajectories_X_output(i,iteration_count),trajectories_Y_output(i,iteration_count),'k*');
    hold on;
end
for i=1:robotCount
    color = colors(j);
    j = j + 1;
    if(j>6)
        j = 1;
    end
    plot(trajectories_X_output(i,:),trajectories_Y_output(i,:),color);
    hold on;
end



%plot final robot positions in third figure
final_pos_X = zeros(1,robotCount);
final_pos_Y = zeros(1,robotCount);
for i=1:robotCount
    final_pos_X(i) = trajectories_X_output(i,iteration_count);
    final_pos_Y(i) = trajectories_Y_output(i,iteration_count);
end
figure;
patch(xb,yb,'white','edgecolor','blue');
hold on;
title('FINAL STATE');
for i=1:robotCount
    [Xsense_final,Ysense_final] = approx_circle(R,[final_pos_X(i);final_pos_Y(i)]);
    %[Xcomms_final,Ycomms_final] = approx_circle(R,[final_pos_X(i);final_pos_Y(i)]);
    plot(final_pos_X(i),final_pos_Y(i),'k.');
    plot(Xsense_final,Ysense_final,'k');
    %plot(Xcomms_final,Ycomms_final,'--r');
    hold on;
end

%plot effective area progression
figure;
%t = 0:dt:dt*iterlim;
t = 1:iteration_count;
plot(t,effective_area_progression_output);
title('Effective area covered');

%display output
fprintf('\nAlgorithm converged after %d',iteration_count);
fprintf(' iteration(s).');
if(termination_condition == 0)
    fprintf(' (Iteration limit reached) \n');
end
fprintf('\nEffective area covered : %f\n',effective_area_progression(iteration_count));

