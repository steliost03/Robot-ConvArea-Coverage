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

%approximates a circle with a polygon (center must be given in vector form)
function [Xcircle,Ycircle] = approx_circle(radius,center)

    angle = 0:0.01:2*pi;
    Xcircle = radius*cos(angle);
    Ycircle = radius*sin(angle);
    xcenter = center(1);
    ycenter = center(2);
    Xcircle = Xcircle + xcenter;
    Ycircle = Ycircle + ycenter;
    
end
