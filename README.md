# Turtlebot-Astar-Path-Planning-with-PID-Kalman-Filter

## Introduction
Please see the 'Turtlebot Path Planning PID Controls Research Report' file for details on the project.
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

- ```decisions.py```
- ```planner.py```
- ```localization.py```
- ```controller.py```
- ```rrt_star.py```
- ```planner.py```

To test RRT*:
- Create a virtual map with virtual obstacles (see the RRT* code ```main``` function). This means that these obstacles will not be visualized in Gazebo nor in RViz (you can do extra work to have them visualized in RViz but this is not necessary and there are no bonus points associated).
- Use the Extended Kalman Filter as localizer, use your tuned covariances, or tune them in simulation. 
- Choose a goal pose: you can use the same RViz interface to choose a goal pose, or you can hard-code a goal pose.
- Execute the path in Gazebo.
- *Log the poses of the robot and covariances from the EKF for the plots.

**To choose a goal pose in RViz**
(Remember to run the map publisher if you are using the map)
- In a terminal run the rviz2 with the given configuration: ```rviz2 -d pathPlanner.rviz```
- In another terminal run the decisions.py: ```python3 decisions.py```
- On rvzi2 use the 2D goal pose on the toolbar on top to choose the goal pose
- Watch the robot go to the specified point

- Tune your RRT* parameters (e.g. circle radius ```connect_circle_dist``` to be considered, expansion distance ```expand_dist```).

## Appendix

**Functions from rrt.py**

The ```rrt_star.py``` file uses some functions from ```rrt.py```, mainly the following ones, you can find them with comments explaining the functions in ```rrt.py```.
- ```steer```
- ```check_collision```

In this case, we are interested in the covariance in the `x` and `y` directions. The axis of the ellipse are defined by the standard deviations $\sigma_x$ and $\sigma_y$ (note standard deviations and not variance), leading to the following equation of the ellipse:

$$\begin{equation}(\frac{x}{\sigma_x})^2 + (\frac{y}{\sigma_y})^2 = d \end{equation}$$

Note that ```d``` is the scale of the ellipse and can be any number, e.g. 1.
Make sure also to orient the ellipse so that it is tangent to the path (oriented as the heading ```theta``` of the robot).
