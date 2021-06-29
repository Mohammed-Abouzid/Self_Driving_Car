# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   


my implementation is in:
- main.cpp (lines 108-189)  and 
- 2 helper fuctions in helper.h (lines 159-265)


firstly I started with getting the car's s position in frenet coordinates and then used the get_fusion() function in helpers.h (lines 159-192) to get the free lanes and if the car too close to the car in front of it or not.

then used if else condition to change the lane or increase the speed, based on the outcom of the get_fusion() function.


lines 135-146, used to get the points needed for our path. I used the get_points() function in helper.h (lines 197-265) to achieve this.

then used spline interpolation to interpolate between the points and get the full path(lines 150-187)


for more details, please have a look at the comments in the files main.cpp and helpers.h
