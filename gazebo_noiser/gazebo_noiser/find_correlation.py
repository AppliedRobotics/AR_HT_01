import math
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import utils
import scipy.stats
class FindCorrelation():
    def __init__(self):
        # print("file name pls ->")
        # self.filename = input()
        self.filename_1 = "simple_turn"
        self.filename_2 = "simple_turn_sim"
        
        self.motion_model = 'simple'
        self.alpha_linear = 0.0
        self.alpha_angular = 0.0
        self.alpha_my_0 = 0.1
        self.alpha_my_1 = 0.3
        self.find_correlation()
        

    def find_correlation(self):
        df_1 = pd.read_csv("csv/"+self.filename_1+".csv")
        df_2 = pd.read_csv("csv/"+self.filename_2+".csv")
        
        lm_1 = df_1['Left_motor'].to_numpy()
        rm_1 = df_1['Right_motor'].to_numpy()
        t_1 = df_1['Time']
        x_1, y_1, theta_1, vx_1, w_1, l_vels_1, r_vels_1 = utils.calculate_odom_from_joints(lm_1, rm_1, t_1)
        
        t_2 = df_2['Time']
        lm_2 = df_2['Left_motor'].to_numpy()
        rm_2 = df_2['Right_motor'].to_numpy()
        x_ideal, y_ideal, theta_ideal, vx_ideal, w_ideal, _, _ = utils.calculate_odom_from_joints(lm_2, rm_2, t_2)
        
        #x_2, y_2, theta_2, vx_2, w_2, _, _ = utils.calculate_odom_from_joints_simple_model(lm_2, rm_2, t_2, 
        #    self.alpha_linear, self.alpha_linear, self.alpha_angular,
        #   self.alpha_angular, self.alpha_angular, self.alpha_angular,
        #    seed = 500)
        x_2, y_2, theta_2, vx_2, w_2, l_vels_2, r_vels_2 = utils.calculate_odom_from_joints_my_model(lm_2, rm_2, t_2, 
        alpha_0 = self.alpha_my_0, alpha_1 = self.alpha_my_1, seed = 500)
        
        # _, _, _, vx_2, w_2 = utils.calculate_odom_from_joints(lm_2, rm_2, t_2)
        # print(len(vx_1))
        # print(len(vx_2))
        vx_1, vx_2 = self.shape_corr(np.asarray(vx_1), np.asarray(vx_2))
        vx_1, vx_ideal = self.shape_corr(np.asarray(vx_1), np.asarray(vx_ideal))
        
        w_1, w_2 = self.shape_corr(np.asarray(w_1), np.asarray(w_2))
        w_1, w_ideal = self.shape_corr(np.asarray(w_1), np.asarray(w_ideal))
        
        x_1, x_2 = self.shape_corr(np.asarray(x_1), np.asarray(x_2))
        y_1, y_2 = self.shape_corr(np.asarray(y_1), np.asarray(y_2))
        theta_1, theta_2 = self.shape_corr(np.asarray(theta_1), np.asarray(theta_2))
        fig, axs = plt.subplots(4)        
        l = np.arange(vx_1.shape[0])
        axs[0].plot(x_1, y_1, label = 'coord robot', linewidth = 1)
        axs[0].plot(x_2, y_2, label = 'coord sim', linewidth = 1)
        axs[0].legend()
        axs[0].grid()
        
        l = np.arange(vx_1.shape[0])
        
        axs[1].plot(l, theta_1, label = 'theta robot', linewidth = 1)
        axs[1].plot(l, theta_2, label = 'theta sim', linewidth = 1)
        axs[1].legend()
        axs[1].grid()

        delta_v_1 = vx_1 - vx_ideal
        delta_v_2 = vx_2 - vx_ideal
        # delta_v_3 = vx_1 - vx_2
        # print(delta_v_3.std())
        # print(delta_v_1.std(), delta_v_2.std())
        print(round(abs(delta_v_1.std() - delta_v_2.std())*1000,5))
        axs[2].plot(l, vx_1, label = 'vx robot', linewidth = 1)
        axs[2].plot(l, vx_2, label = 'vx sim', linewidth = 1)
        # axs[2].plot(l, delta_v_1, label = 'vx noise', linewidth = 1)
        axs[2].legend()
        axs[2].grid()

        delta_w_1 = w_1 - w_ideal
        delta_w_2 = w_2 - w_ideal
        print(round(abs(delta_w_1.std() - delta_w_2.std())*1000, 5))
        axs[3].plot(l, w_1, label = 'w robot', linewidth = 1)
        axs[3].plot(l, w_2, label = 'w sim', linewidth = 1)
        axs[3].legend()
        axs[3].grid()
        l = np.arange(l_vels_1.shape[0])
        #axs[4].plot(l, l_vels_1, label = 'l vel robot', linewidth = 1)
        #axs[4].plot(l, l_vels_2, label = 'l vel sim', linewidth = 1)
        #axs[4].legend()
        #axs[4].grid()
        #l = np.arange(l_vels_1.shape[0])
        #axs[5].plot(l, r_vels_1, label = 'r vel robot', linewidth = 1)
        #axs[5].plot(l, r_vels_2, label = 'r vel sim', linewidth = 1)
        #axs[5].legend()
        #axs[5].grid()


        # r = scipy.stats.pearsonr(vx_1, vx_2)
        # print(r)
        # r_w = scipy.stats.pearsonr(w_1, w_2)
        # print(r_w)
        #plt.show()
        # print(len(vx_2))


    def shape_corr(self, arr1, arr2):
        if arr1.shape[0] > arr2.shape[0]:
            arr1 = arr1[0:arr2.shape[0]]
            # s_1 = (arr1.shape[0]-arr2.shape[0])/2
            # s_2 = (arr1.shape[0]-arr2.shape[0])
            # arr1 = arr1[arr1.shape[0]-arr2.shape[0]:]
        else:
            # arr2 = arr2[arr2.shape[0]-arr1.shape[0]:]
            arr2 = arr2[0:arr1.shape[0]]
        return arr1, arr2

   


def main():
    pff = FindCorrelation()

if __name__ == '__main__':
    main()
