from math import sin, cos
import random
import numpy as np

def calculate_vel_from_joints(l_vel, r_vel):
    BASELINE = 0.430
    WHEELRADIUS = 0.115
    vel = (WHEELRADIUS)*(l_vel+r_vel)/2
    Vx = vel
    Vy = 0
    Vtheta = (WHEELRADIUS)*(l_vel-r_vel)/BASELINE
    return Vx, Vtheta

def calculate_odom_from_joints(l_vels, r_vels, t):
    x = 0
    y = 0
    theta = 0
    x_l = []
    y_l = []
    theta_l = []
    vx_l = []
    w_l = []
    for i in range(1, l_vels.shape[0]):
        delta_t = t[i] - t[i-1]
        vx, w = calculate_vel_from_joints(l_vels[i], r_vels[i])
        vx_l.append(vx)
        w_l.append(w)
        theta +=delta_t*w
        x += delta_t*cos(theta)*vx
        y += delta_t*sin(theta)*vx
        x_l.append(x)
        y_l.append(y)
        theta_l.append(theta)
    return x_l, y_l, theta_l, vx_l, w_l, l_vels, r_vels

def calculate_odom_from_joints_simple_model(l_vels, r_vels, t, a_1, a_2, a_3, a_4, a_5, a_6, seed):
    x = 0
    y = 0
    theta = 0
    x_l = []
    y_l = []
    theta_l = []
    vx_l = []
    w_l = []
    random.seed(seed)
    for i in range(1, l_vels.shape[0]):
        delta_t = t[i] - t[i-1]
        vx, w = calculate_vel_from_joints(l_vels[i], r_vels[i])
        vel_x = vx + get_random_vel(vx, w, a_1, a_2, seed)
        w_z = w + get_random_vel(vx, w, a_3, a_4, seed)
        vx_l.append(vel_x)
        w_l.append(w_z)
        gamma = get_random_vel(vx, w, a_5, a_6, seed)        
        theta +=delta_t*w_z + gamma*delta_t
        x += delta_t*cos(theta)*vx
        y += delta_t*sin(theta)*vx
        x_l.append(x)
        y_l.append(y)
        theta_l.append(theta)
    return x_l, y_l, theta_l, vx_l, w_l, l_vels, r_vels

def get_random_vel(vel, w, alpha_0, alpha_1, seed):
        
        sigma = vel**2 * alpha_0 + w**2 * alpha_1
        rand_sum = 0
        for i in range (0,12):
            rand_sum += random.gauss(0, sigma)
        err = rand_sum/2
        return err


def calculate_odom_from_joints_my_model(l_vels, r_vels, t, alpha_0, alpha_1, seed):
    x = 0
    y = 0
    theta = 0
    x_l = []
    y_l = []
    theta_l = []
    vx_l = []
    w_l = []
    l_vels_ret = []
    r_vels_ret = []
    prev_l_vel = 0
    prev_r_vel = 0
    # random.seed(seed)
    for i in range(1, l_vels.shape[0]):
        delta_t = t[i] - t[i-1]
        if i != 0:
            l_vel = l_vels[i] + random.gauss(0, (l_vels[i] - prev_l_vel)*alpha_0) + random.gauss(0, l_vels[i]*alpha_1)
            r_vel = r_vels[i] + random.gauss(0, (r_vels[i] - prev_r_vel)*alpha_0) + random.gauss(0, r_vels[i]*alpha_1)
            # print(l_vels[i] - prev_l_vel)
        else:
            l_vel = l_vels[i]
            r_vel = r_vels[i]
        l_vels_ret.append(l_vel)
        r_vels_ret.append(r_vel)
        vx, w = calculate_vel_from_joints(l_vel, r_vel)
        vx_l.append(vx)
        w_l.append(w)
        theta +=delta_t*w
        x += delta_t*cos(theta)*vx
        y += delta_t*sin(theta)*vx
        x_l.append(x)
        y_l.append(y)
        prev_l_vel = l_vel
        prev_r_vel = r_vel
        theta_l.append(theta)
    return x_l, y_l, theta_l, vx_l, w_l, np.asarray(l_vels_ret), np.asarray(r_vels_ret)
