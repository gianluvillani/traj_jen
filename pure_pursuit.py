"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""


import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from car_gar import *

        
    
 


k = 0.05  # look forward gain
Lfc = 0.1  # look-ahead distance
Kp = 2.0  # speed proportional gain
dt = 0.01  # [s]
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
    phi_min=-0.3*math.pi
    phi_max=0.3*math.pi
    phi=np.random.rand()*(phi_max-phi_min)-(phi_max+phi_min)/2
    #rip
    goal_x=math.cos(phi)+curr_goal_dot.x
    goal_y=math.sin(phi)+curr_goal_dot.y
    return [goal_x,goal_y]
    
    
    
    
def pure_pursuit_sim(x_0, y_0, yaw_0, v_0, cx=0, cy=0):
    #  target course
    point_1 = dots(x=0, y=0, v=0.1, a=0.0, yaw=math.pi/4)
    point_2 = dots(x=1, y=0.7, v=0.1, a=0.2, yaw = 0.8*math.pi/4)
    point_3 = dots(x=2, y=0.3, v=0.1, a=0.2, yaw = 0.8*math.pi/4)
    point_4 = dots(x= 3, y=1.2, v=0.1, a=0.2,  yaw = 0.8*math.pi/4)
    point_5 = dots(x = 4, y=0.3, v=0.1, a=0.2, yaw = -0.8*math.pi/4)
    point_6 = dots(x=5, y=0, v=0.1, a=0.2, yaw = -math.pi/4)
    waypoints = [point_1, point_2, point_4, point_5, point_6]
    # Generate the car and initial trajectory
    car_1 = car(x=x_0, y=y_0, v=v_0, a=0,yaw=0, dist_goal=0.5, dotlist=waypoints)
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
    target_speed = 6.0 / 3.6  # [m/s]

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
#        x.append(state.x)
#        y.append(state.y)
#        yaw.append(state.yaw)
#        v.append(state.v)
#        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(state.x, state.y, "D", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.01)
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


