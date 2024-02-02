# Turtlebot-Astar-Path-Planning-with-PID-Kalman-Filter

## Introduction
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
- Use the Extended Kalman Filter as localizer, use your tuned covariances, or tune them in simulation, we will not deduct marks for poor localization performance. 
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
- Plot containing the virtual obstacles, the obtained path (optionally the entire RRT* tree), clearly marking the starting and goal positions. Overlay the generated path with the executed path. To plot the covariances as ellipses (see Appendix). 

* Section 1 - Stack: the description and discussion of the stack (see Part 1), max 500 words.
* Section 2 - RRT* implementation: 
  * Describe how th provided RRT* works and which modifications you implemented.
  * Describe how you integrated the path planning and navigation for the robot from the planner to the actual motions of the robot (you do not need to describe the entire stack again as you should have done this in Part 1), including the implementation of path smoothing.
* Section 3 - Testing: 
  * Report figures as specified in Part 3.
  * Discuss how you tuned your RRT* parameters to reach the final parameters you used for the plots.
  * Discuss the performance of your RRT*.
* Section 4 - Final discussions:
  * Discuss the overall performance of your entire stack, i.e. including your PID controller, EKF, and RRT* planner.

## Appendix

**Functions from rrt.py**

The ```rrt_star.py``` file uses some functions from ```rrt.py```, mainly the following ones, you can find them with comments explaining the functions in ```rrt.py```. It is advised to get familiar with them before proceeding with the completion of the code:
- ```steer```
- ```check_collision```

In this case, we are interested in the covariance in the `x` and `y` directions. The axis of the ellipse are defined by the standard deviations $\sigma_x$ and $\sigma_y$ (note standard deviations and not variance), leading to the following equation of the ellipse:

$$\begin{equation}(\frac{x}{\sigma_x})^2 + (\frac{y}{\sigma_y})^2 = d \end{equation}$$

Note that ```d``` is the scale of the ellipse and can be any number, e.g. 1.
Make sure also to orient the ellipse so that it is tangent to the path (oriented as the heading ```theta``` of the robot).

# Turtlebot-Astar-Path-Planning-with-PID-Kalman-Filter
# Turtlebot-Astar-Path-Planning-with-PID-Kalman-Filter
