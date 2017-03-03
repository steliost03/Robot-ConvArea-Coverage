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

%called by BoundedVoronoi() function.
function [Xplane,Yplane] = form_plane(sourcePoint,otherPoint)
    
    large_value = 10000;
    vector_between_points = [sourcePoint otherPoint];
    rotation90_mat = [0 -1;1 0 ];
    
    bisector_midpoint_x = (sourcePoint(1)+otherPoint(1))/2;
    bisector_midpoint_y = (sourcePoint(2)+otherPoint(2))/2;
    
    perp_vector = rotation90_mat*vector_between_points;
    perp_vector_i = perp_vector(1,2)-perp_vector(1,1);
    perp_vector_j = perp_vector(2,2)-perp_vector(2,1);
    vec_bpoints_i = vector_between_points(1,2) - vector_between_points(1,1);
    vec_bpoints_j = vector_between_points(2,2) - vector_between_points(2,1);
    
    point1_x = bisector_midpoint_x + large_value*perp_vector_i;
    point1_y = bisector_midpoint_y + large_value*perp_vector_j;
    
    point2_x = point1_x - large_value*vec_bpoints_i;
    point2_y = point1_y - large_value*vec_bpoints_j;
    
    point3_x = bisector_midpoint_x - large_value*perp_vector_i;
    point3_y = bisector_midpoint_y - large_value*perp_vector_j;
    
    point4_x = point3_x - large_value*vec_bpoints_i;
    point4_y = point3_y - large_value*vec_bpoints_j;
    
    %clockwise ordering
    Xplane = [point1_x point3_x point4_x point2_x];
    Yplane = [point1_y point3_y point4_y point2_y];
    
end   
