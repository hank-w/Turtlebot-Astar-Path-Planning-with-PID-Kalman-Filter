from mapUtilities import *
from a_star import *
import time
from rrt_star import *
from utilities import *
import matplotlib as plot
from scipy.interpolate import splprep, splev
import rclpy

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.startPose = startPose
        self.endPose = endPose
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        # self.m_utilites=mapManipulator(laser_sig=0.4)    
        # self.costMap=self.m_utilites.make_likelihood_field()

    #     # obstacle 1
    #     obstacle_list = [
    #     (5, 5, 1),
    #     # (3, 6, 2),
    #     # (3, 8, 2),
    #     (3, 10, 2),
    #     (7, 5, 2),
    #     (9, 5, 2),
    #     (8, 10, 1),
    #     (6, 12, 1),
    # ]  # [x,y,size(radius)]
        
        # obstacle 2
        obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        # (8, 10, 1),
        # (6, 12, 1),
    ]  # [x,y,size(radius)]
        
        self.rrtStar = RRTStar(
        start=self.startPose,
        goal=list([7.0,8.0]),
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.8)
        # self.rrtStar = RRTStar(
        # start=[0, 0],
        # goal=[6, 10],
        # rand_area=[-2, 15],
        # obstacle_list=obstacle_list,
        # expand_dis=1,
        # robot_radius=0.8)

        self.rrt = RRT(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.8)
        
    
    def trajectory_planner(self, startPoseCart, endPoseCart, type):
        
        #### If using the map, you can leverage on the code below originally implemented for A*
        #### If not using the map, you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)
        # scale_factor = 1 # this is the downsample scale, if set 2, it will downsample the map by half, and if set x, it will do the same as 1/x


        # startPose=self.m_utilites.position_2_cell(startPoseCart)
        # endPose=self.m_utilites.position_2_cell(endPoseCart)
        

        start_time = time.time()
        
        # startPose = [int(i/scale_factor) for i in startPose]
        # endPose   = [int(j/scale_factor) for j in endPose]

        # mazeOrigin = self.m_utilites.position_2_cell([0,0])

        #  This is for A*, modify this part to use RRT*
        # path = search(self.costMap, startPose, endPose, scale_factor)

        path = self.rrtStar.planning(animation=True)
        end_time = time.time()
        try:
            path = self.interpolate(3,path[0:-1])
        except:
            print("No Valid Path Found, Please Rerun Path Planning")
        path.reverse()
        # path = self.rrt.planning()
        
        print(f"the time took for rrt_star calculation was {end_time - start_time}")
        # DONE Apply smoothing to path
        # path = smoothing(path)


  

        # This will display how much time the search algorithm needed to find a path


        # path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]
        # Path = np.array(list(map(self.m_utilites.cell_2_position, path_ )))

        #  Smooth the path before returning it to the decision maker
        # this can be in form of a function that you can put in the utilities.py 
        # or add it as a method to the original rrt.py 
        np_path = np.array(path)
        smooth = self.smoothing(np_path)
        return smooth
    
    def interpolate(self, t, path):
        res = []
        res.append(path[0])
        for idx, val in enumerate(path):
            if idx == 0:
                continue
            cost = int(sqrt( (path[idx][1] - path[idx-1][1])**2 + (path[idx][0] - path[idx-1][0])**2 ))
            if cost >= t:
                set_size = int(cost) // int(t)
                res_set = np.linspace(path[idx-1],path[idx],set_size+2)
                res_set = res_set[1:-1]
                res.extend(res_set)
            res.append(path[idx])
        return res
    
    def smoothing(self,path):
        if len(path) < 4:
            return path
        x = [path[0][0]]
        y = [path[0][1]]
        for idx in range(1,len(path)):
            if path[idx][0] != path[idx-1][0] and path[idx][1] != path[idx-1][1]:
                x.append(path[idx][0])
                y.append(path[idx][1])
        x = np.array(x)
        y = np.array(y)
        splev_input1,temp = splprep([x,y],s=0.6)
        splev_input2 = np.linspace(0,1,800)
        smoothedx,smoothedy = splev(splev_input2,splev_input1)
        res_path = np.column_stack((smoothedx,smoothedy))
        
        plt.plot(x, y, 'o-', label='Path')
        plt.plot(smoothedx, smoothedy, label='Smoothed Path')
        plt.title('Smoothing Function Applied on Path')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.legend()
        plt.show()
        return res_path



if __name__=="__main__":
    rclpy.init()

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can do your test here ...

