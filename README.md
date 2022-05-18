# Automated-parking

A common path planning problem for autonomous vehicles involves maneuvering in tight
spaces and cluttered environments, particularly while parking. in this project I have created the simulated world and
park three vehicles of increasing ridiculousness into a compact space. Planners take into
account vehicle kinematics and collision.

In this problem I have used Hybrid A* algorithm to a nonholonomic mobile outdoor robot in
order to plan near optimal paths. Unlike the regular A*, the Hybrid A* algorithm is capable of
taking into account the continuous nature of the search space representing the real world.
Controller pure pursuit is used here. The controllerPurePursuit System object in
MATLAB creates a controller object used to make a differential-drive vehicle follow a set of
waypoints. The object computes the linear and angular velocities for the vehicle given the current
pose.

## Results

![image](https://user-images.githubusercontent.com/94932358/169156738-c93f93c8-e16b-49df-bc21-0ab595d896dd.png)


https://user-images.githubusercontent.com/94932358/169156461-4bff6b98-0f93-4fe6-a40f-5e8333119dcb.mp4

