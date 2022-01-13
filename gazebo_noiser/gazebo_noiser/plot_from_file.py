import math
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import utils
class PlotFromFile():
    def __init__(self):
        print("file name pls ->")
        # self.filename = input()
        self.filename = "acc_test_sim"
        utils.calculate_vel_from_joints(0,0)
        # self.plot_odom()
        self.motion_model = 'no'
        self.alpha_linear = 0.1
        self.alpha_angular = 0.3
        self.my_alpha_0 = 0.2
        self.my_alpha_1 = 0.15
        self.plot_all()
        

    def plot_all(self):
        df = pd.read_csv("csv/"+self.filename+".csv")
        lm = df['Left_motor'].to_numpy()
        rm = df['Right_motor'].to_numpy()
        fig, axs = plt.subplots(4)        
        
        t = df['Time']
        if self.motion_model == 'no':
            x, y, theta, vx, w, _ ,_ = utils.calculate_odom_from_joints(lm, rm, t)
        elif self.motion_model == 'simple':
            x, y, theta, vx, w, _, _= utils.calculate_odom_from_joints_simple_model(lm, rm, t, 
            self.alpha_linear, self.alpha_linear, self.alpha_angular,
            self.alpha_angular, self.alpha_angular, self.alpha_angular,
            seed = 100)
        elif self.motion_model == 'my':
            x, y, theta, vx, w, lm, rm = utils.calculate_odom_from_joints_my_model(lm, rm, t,
             alpha_0 = self.my_alpha_0, alpha_1 = self.my_alpha_1 ,seed = 2231) 

        size_ = lm.shape[0]
        x__ = np.arange(size_)
        axs[0].plot(x__, lm, label = 'left motor', linewidth = 1)
        axs[0].plot(x__, rm, label = 'right motor', linewidth = 1)
        axs[0].set_xlabel("iteration")
        axs[0].set_ylabel("radians/sec")
        axs[0].legend()
        axs[0].grid()



        count = np.arange(len(vx))
        axs[1].plot(count, vx, label = 'linear speed', linewidth = 1)
        axs[1].plot(count, w, label = 'angular speed', linewidth = 1)
        axs[1].set_xlabel("X")
        axs[1].set_ylabel("speed")
        axs[1].legend()
        axs[1].grid()
        

        axs[2].plot(x, y, label = 'odom', linewidth = 1)
        axs[2].set_xlabel("X")
        axs[2].set_ylabel("y")
        axs[2].legend()
        axs[2].grid()
        x_ = np.arange(len(theta))
        axs[3].plot(x_, theta, label = 'orientation', linewidth =1)
        axs[3].set_xlabel("iteration")
        axs[3].set_ylabel("alpha")
        axs[3].legend()
        axs[3].grid()
        plt.show()



    def plot_js(self):
        df = pd.read_csv("csv/"+self.filename+".csv")
        lm = df['Left_motor'].to_numpy()
        rm = df['Right_motor'].to_numpy()
        # rm = rm*(-1)
        size_ = lm.shape[0]
        x = np.arange(size_)
        fig, ax = plt.subplots()
        ax.plot(x, lm, label = 'left motor', linewidth = 1)
        ax.plot(x, rm, label = 'right motor', linewidth = 1)
        ax.set_xlabel("X")
        ax.set_ylabel("y")
        ax.legend()
        plt.grid()
        plt.show()

    def plot_odom(self):
        df = pd.read_csv("csv/"+self.filename+".csv")
        lm = df['Left_motor'].to_numpy()
        rm = df['Right_motor'].to_numpy()
        t = df['Time']
        x, y, theta = utils.calculate_odom_from_joints(lm, rm, t)
        fig, axs = plt.subplots(2)
        axs[0].plot(x, y, label = 'odom', linewidth = 1)
        axs[0].set_xlabel("X")
        axs[0].set_ylabel("y")
        axs[0].legend()
        axs[0].grid()
        x_ = np.arange(len(theta))
        axs[1].plot(x_, theta, label = 'orientation', linewidth =1)
        axs[1].set_xlabel("X")
        axs[1].set_ylabel("alpha")
        axs[1].legend()
        axs[1].grid()
        plt.show()

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z # in radians

def main():
    pff = PlotFromFile()

if __name__ == '__main__':
    main()
