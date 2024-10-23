# Type of planner
import math
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

PARABOLA = 0
SIGMOID = 1
TRAJ = PARABOLA

tol = 1e-3

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    def generateTraj(self, f, range, step):
        traj = []
        x = range[0]
        while x <= range[-1] + tol:
            traj.append([x, f(x)])
            x += step
        return traj

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        if TRAJ == PARABOLA:
            parabola = lambda x: x**2
            return self.generateTraj(parabola, [0, 1.5], 0.1)
        elif TRAJ == SIGMOID:
            sigmoid = lambda x: 2/(1+math.exp(-2*x)) - 1
            return self.generateTraj(sigmoid, [0, 2.5], 0.1)
        else:
            raise Exception("invalid trajectory type given")
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

