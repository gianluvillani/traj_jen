"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""


import numpy as np
import math
import matplotlib.pyplot as plt

from scipy.interpolate import splev, splrep
import quintic_polynomials_planner as qp
from pycubicspline import * 
from matplotlib import animation

class car:
    def __init__(self, x, y, v, a, yaw, dotlist, dist_goal):
        self.carstate = dots(x, y, v, a, yaw)
        self.traj = trajectory(dotlist, dist_goal)
        self.traj.update_start(self.carstate)
        self.traj.gen_trajectory()
    
    def move_car(self, x, y, v, yaw, new_points):
        self.x = x
        self.y = y
        self.v = v
        self.yaw = yaw
        self.traj.update_start(self.carstate) # updated only when the car is close enough to the closest waypoint
        if new_points:
            self.traj.gen_trajectory() # recompute the trajectory to the actual state
    
    
    def reached_goal():
        return traj.reached_final_goal()
    
    def add_goal(next_goal):
        self.trajectory.update_goal(next_goal)

class dots:
    def __init__(self, x, y, v, yaw, a =0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        #self.a = a
        self.vx = v*math.cos(yaw)
        self.vy = v*math.sin(yaw)
        #self.ax = a*math.cos(yaw)
        #self.ay = a*math.sin(yaw)
        
        
    def update_state(self,x, y, v, a, yaw):
        """
        Update coordinates for the car
        """
        self.x=x
        self.y=y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.vx = v*math.cos(yaw)
        self.vy = v*math.sin(yaw)
        self.ax = a*math.cos(yaw)
        self.ay = a*math.sin(yaw)
        
        
        
        
class trajectory:
    def __init__(self, dotlist, close_dist):
            self.dotlist=dotlist #Dots to be trajectorized
            self.dist = close_dist #Maximum distance
                                   # between the car state and the closest waypoint 
    
    def update_goal(self, dot_goal):
        """
        Called each time a new goal point is added
        """
        self.dotlist.append(dot_goal)   
    def close_enough(self, car_state, curr_dot):
        """
        Check if car has passed a dot in the trajectory
        """
        print(curr_dot.x)
        print(curr_dot.y)
        print(car_state.x)
        print(car_state.y)
        dist = math.sqrt((curr_dot.x-car_state.x)**2+(curr_dot.y-car_state.y)**2)
        return dist < self.dist
    
    def update_start(self, car_state):
        self.dotlist[0] = car_state
        if self.close_enough(car_state, self.dotlist[1]):
            for i in self.dotlist:
                print(i.x)
                print(i.y)
            print('ja')
            del self.dotlist[1]
            return True
        
        return False    
    def gen_trajectory(self):
        """
        Trajectory generator
        """
        # do stuff here
        #x_trajectory = linspace(self.start, self.) 
        dt = 0.01
        max_accel = 10
        self.x_ref = []
        self.y_ref = []
        self.v_ref = []
        self.a_ref = []
        self.yaw_ref = []
        mode = 1
        if mode == 0:
            for i in range(len(self.dotlist)-1):


                curr_dot = self.dotlist[i]

                goal_dot = self.dotlist[i+1]  
                print(goal_dot.x)
                # Get the point coordinate, the first one is the car position

                [time,rx,ry,ryaw,rv,ra] = qp.quinic_polynomials_planner(sx=curr_dot.x, sy=curr_dot.y, sv=curr_dot.v,
                                                                        syaw=curr_dot.yaw, sa=curr_dot.a, gx=goal_dot.x,
                                                                        gy = goal_dot.y, gv = goal_dot.v, ga = goal_dot.a,
                                                                       gyaw = goal_dot.yaw, dt = dt, max_accel = max_accel)


                self.x_ref.extend(rx)
                self.y_ref.extend(ry)
                self.v_ref.extend(rv)
                self.a_ref.extend(ra)
                self.yaw_ref.extend(ryaw)
        if mode == 1:
            x = []
            y = []
            
            
            for i in range(len(self.dotlist)):
                x.append(self.dotlist[i].x)
                y.append(self.dotlist[i].y)
            
            sp = Spline2D(x, y)
            s = np.arange(0, sp.s[-1], 0.1)
            rx, ry, ryaw, rk = [], [], [], []
            for i_s in s:
                ix, iy = sp.calc_position(i_s)
                rx.append(ix)
                ry.append(iy)
                ryaw.append(sp.calc_yaw(i_s))
                rk.append(sp.calc_curvature(i_s))
            
            self.x_ref = rx
            self.y_ref = ry
            self.yaw_ref = ryaw
            
            
            
            
    def get_next_traj(self):
        return dots(x_vec[0], y_vec[0], 0)
    
    def reached_final_goal(self):
        return close_enough(dotlist[0], dotlist[-1])
        
    def return_goal(self):
        return self.dotlist[-1]
        

        
    
 


k = 0.1  # look forward gain
Lfc = 0.01  # look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.02  # [s]
L = 2.9  # [m] wheel base of vehicle


show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def PIDControl(target, current): # PID for acceleration
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d)) # find the closest point in the trajectory
    L = 0.0

    Lf = k * state.v + Lfc # Lf is like the 'maximum'

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def generate_semi_random_point(r, curr_goal_dot):
    """
    Semi random dot generator for simulation purposes
    """
    phi_min=-0.5*math.pi
    phi_max=0.5*math.pi
    phi=np.random.rand()*(phi_max-phi_min)-(phi_max+phi_min)/2
    #rip
    goal_x=math.cos(phi)+curr_goal_dot.x
    goal_y=math.sin(phi)+curr_goal_dot.y
    return [goal_x,goal_y]
    
    
    
    
def pure_pursuit_sim(x_0, y_0, yaw_0, v_0, cx=0, cy=0):
    #  target course
    point_1 = dots(x=0, y=0, v=0.1, a=0.0, yaw=math.pi/4)
    point_2 = dots(x=0.3, y=0.7, v=0.1, a=0.2, yaw = 0.8*math.pi/4)
    point_3 = dots(x=0.5, y=0.3, v=0.1, a=0.2, yaw = 0.8*math.pi/4)
    point_4 = dots(x= 1, y=1.2, v=0.1, a=0.2,  yaw = 0.8*math.pi/4)
    point_5 = dots(x = 2.3, y=0.3, v=0.1, a=0.2, yaw = -0.8*math.pi/4)
    point_6 = dots(x=3, y=0, v=0.1, a=0.2, yaw = -math.pi/4)
    waypoints = [point_1, point_2, point_4]
    # Generate the car and initial trajectory
    car_1 = car(x=x_0, y=y_0, v=v_0, a=0,yaw=0, dist_goal=0.3, dotlist=waypoints)
    traj = car_1.traj
    cx = traj.x_ref
    cy = traj.y_ref

    len_before = len(cx)
    diff = 0
    plt.plot(cx, cy)
    # 
    
    
    
    # things to modify inside
    
    '''
    cx, cy -> reference trajectories should be changed dynamically as soon as
    new points are added.
    
    
    '''
    target_speed = 3.0 / 3.6  # [m/s]

    T = 2000.0  # max simulation time

    # initial state
    state = State(x=x_0, y=y_0, yaw=yaw_0, v=v_0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
    provola = 0
    while True:
        ai = PIDControl(target_speed, state.v) # acceleration command
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        # update states with the control signals computed above
        state = update(state, ai, di)
        car_1.carstate.update_state(x= state.x, y= state.y, v=state.v, a=0, yaw=state.yaw)
        new_points = car_1.traj.update_start(car_1.carstate)           
        car_1.move_car(x = state.x, y = state.y, v = state.v, yaw = state.yaw, new_points= new_points)
        provola = provola +1
        if new_points:
            r = 1
            [goal_x,goal_y]=generate_semi_random_point(r, car_1.traj.return_goal())
            car_1.traj.update_goal(dots(x = goal_x, y = goal_y, v =0, yaw =0))
            car_1.traj.gen_trajectory()
            cx = traj.x_ref
            cy = traj.y_ref
            len_after = len(cx)
            target_ind = calc_target_index(state, cx, cy)
            di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
            #target_speed = 0
            
        time = time + dt
        
        #check car close to first dot update dot list.
        
        
        #
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
        #print('time elapsed')
        #print(T >= time)
        #print('last index')
        #print(lastIndex > target_ind)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


