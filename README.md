# Robot-ConvArea-Coverage

## Overview

Simulation for area coverage of a convex environment by N ground robots.

The area coverage of each robot is circular , with a constant radius (common between all robots).

Each robot also has a constant (and also common in value with the others) communication radius , within which they can sense the existence 
of other robots.

The control law implemented is the **r-limited voronoi cell centroid** method.

## Screenshots

<img src="https://cloud.githubusercontent.com/assets/20325266/23537679/6e8cc8f0-ffd6-11e6-84f3-5585be9b0836.jpg" width="49%"><img src="https://cloud.githubusercontent.com/assets/20325266/23537683/755f027e-ffd6-11e6-861a-9768f6cc7252.jpg" width="49%">

<img src="https://cloud.githubusercontent.com/assets/20325266/23537681/72e2d9b2-ffd6-11e6-9ee0-af295233a2a5.jpg" width="49%"><img src="https://cloud.githubusercontent.com/assets/20325266/23537684/76a5783e-ffd6-11e6-8538-da219c29d3d4.jpg" width="49%">


## How to run

*SIMULATION.m* is the main file from which the simulation is executed.

The rest of the files implement the various functions that are used (see below for details) .

## Parameters

The following parameters can be determined by the user :

**xc** and **yc**: Vectors containing the vertex coordinates of the convex environment.

**px** and **py** : Vectors containing the coordinates for the initial robot positions.

**R** : The area sensing radius of the robots.

**Rc** : The communication radius of the robots.

**K** : Control law gain

**dt** : The fundamental time unit. Each iteration will represent dt seconds passed.

**thr** : Convergence threshold .
If the next-iteration centroid changes less than this value for ALL robots, then the algorithm terminates.

**iterlim** : The maximum amount of iterations allowed.

**robotCount** : The number of ground robots active in the environment.

## How it works

For each robot,the following procedure is executed repeatedly , until the algorithm converges or the iteration limit is reached :

* Calculate the bounded voronoi cell of the robot. The bounded voronoi cell is the  intersection  of the regular (not r-limited)
voronoi cell with the environment . Note that in the calculation of the voronoi cell, only the robots that are inside the current
robot's communication radius are taken into account.

* Calculate the r-limited voronoi cell of the robot . The r-limited voronoi cell is produced by the intersection of the bounded voronoi
with the sensing circle of the robot.
 
* Calculate the centroid of the r-limited voronoi cell.

* Calculate the output velocity of the robot, which is proportionate to its distance from the centroid (multiplied by the gain factor).


## Functions

**BoundedVoronoi()**  

Calculates the regular (not r-limited) voronoi cell for a robot (using an approximate half-plane intersection method) 
and finds its intersection with the convex environment. This is then named as its bounded voronoi cell.


**form_plane()**  

Approximates a half-plane via a sufficiently large polygon. This function is called by BoundedVoronoi().


**Control_Law()** 

For a given robot with a given bounded voronoi cell ,and a given sensing radius :

(1) Finds its r-limited voronoi cell

(2) Calculates the centroid of the r-limited voronoi cell

(3) Calculates its velocity vector, which is determined by its distance from the centroid (and the gain factor)


**approx_circle()**  

Approximates a circle via a polygon with sufficiently large amount of vertices. This function is used to 
represent the sensing circle and communication radius of each robot.


## Dependencies

This simulation uses *polygeom* for the calculation of polygon centroids. It is licensed under the BSD License.

(https://www.mathworks.com/matlabcentral/fileexchange/319-polygeom-m)



## License

This simulation is distributed under Apache License Version 2.0

Copyright (C) 2017 Stylianos Tsiakalos
