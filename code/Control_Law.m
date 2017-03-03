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

%robotPosition -> column vector [x;y]
%Xcell,Ycell -> polygon representing voronoi cell
%dt -> the amount of time each iteration represents
function newRobotPosition = Control_Law(robotPosition,Xcell,Ycell,Xsense,Ysense,gainFactor,dt)

    %step 1: find the centroid of the intersection of voronoi-sensing area
    [Xarea,Yarea] = polybool('intersection',Xcell,Ycell,Xsense,Ysense);
    [temp,~,~] = polygeom(Xarea,Yarea);
    centroid_x = temp(2);
    centroid_y = temp(3);
    
    
    %step 2: find distance of robot from centroid
    dist_x = abs(centroid_x - robotPosition(1));
    dist_y = abs(centroid_y - robotPosition(2));
    
    if(robotPosition(1) > centroid_x)
        dist_x = -dist_x;
    end
    if(robotPosition(2) > centroid_y)
        dist_y = -dist_y;
    end
    
    
    %step 3: use control law to find output velocity
    
    vel_magnitude(1) = gainFactor*dist_x;
    vel_magnitude(2) = gainFactor*dist_y;
    
    dx = vel_magnitude(1)*dt;
    dy = vel_magnitude(2)*dt;
    
    newRobotPosition_X = robotPosition(1)+dx;
    newRobotPosition_Y = robotPosition(2)+dy;
    
    newRobotPosition = [newRobotPosition_X;newRobotPosition_Y];
    
end
