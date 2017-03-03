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

%sourcePoint -> column vector [x;y]
%other Points -> 2xN matrix, N = point count
%Xboundary,Yboundary -> coordinates of environment boundary edges.
%Rc -> communication range for every point (assumes circular area)
function [Xcell,Ycell] = BoundedVoronoi(sourcePoint,sourceIndex,allPoints,Xboundary,Yboundary,Rc)
    
    pointCount = length(allPoints);
    
    
    points_in_range = 0;
    
    %step 0 : find how many other points are in comm.range
    for i=1:pointCount
        
        if(i == sourceIndex)
            continue;
        end
        
        dist_x = sourcePoint(1) - allPoints(1,i);
        dist_y = sourcePoint(2) - allPoints(2,i);
        dist = sqrt(dist_x*dist_x + dist_y*dist_y);
        if(dist <= Rc)
            points_in_range = points_in_range + 1;
        else
            continue;
        end
        
    end
    
    if(points_in_range == 0)
        Xcell = Xboundary;
        Ycell = Yboundary;
        return;
    end
    otherPointsCount = points_in_range;
    Xplanes = cell(1,otherPointsCount);
    Yplanes = cell(1,otherPointsCount);
    
    %step 1 : calculate the planes for each other point
    counter = 1;
    for i=1:pointCount
       
        if(i == sourceIndex)
            %avoid degenerate case
            continue
        end
        
        dist_x = sourcePoint(1) - allPoints(1,i);
        dist_y = sourcePoint(2) - allPoints(2,i);
        dist = sqrt(dist_x*dist_x + dist_y*dist_y);
        if(dist > Rc)
           continue;
        end
        
        [Xplanes{1,counter},Yplanes{1,counter}] = form_plane(sourcePoint,allPoints(:,i));
        counter = counter + 1;
          
    end
    
    
    %step 2 : calculate the intersection of all the planes for each robot
    Xintersec = Xplanes{1,1};
    Yintersec = Yplanes{1,1};
  
    for i=2:otherPointsCount
        X_current_plane = Xplanes{1,i};
        Y_current_plane = Yplanes{1,i};
        [Xintersec,Yintersec] = polybool('intersection',X_current_plane,Y_current_plane,Xintersec,Yintersec);
    end
    
  
    XunboundedCell = Xintersec;
    YunboundedCell = Yintersec;
    
    %step 3 : limit the voronoi cells to the given environment 
    [Xcell,Ycell] = polybool('intersection',XunboundedCell,YunboundedCell,Xboundary,Yboundary);
   
end
