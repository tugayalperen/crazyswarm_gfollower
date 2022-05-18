#!/usr/bin/env python

from pycrazyswarm import Crazyswarm
from pos_fb import pos_fb_updater
import numpy as np
import scipy.io as sio
import rospy
import socket
from copy import deepcopy
import math
from signal import signal, SIGINT
import sys
import uav_trajectory

class master_swarm:

    def __init__(self,num_cf, if_heading, if_desired):

        global cf_positions
        global headings

        cf_positions = np.full([3,num_cf],0.01)

        self.map = sio.loadmat('/home/tugay/Environments/linear_sym_4x65.mat')
        self.map = self.map['I']
        self.size_x = 6.5
        self.size_y = 4
        # self.grad_const = (len(np.arange(start = 0.00,stop = self.size ,step=0.0250/(20/self.size)))) / self.size
        self.grad_const_x = (len(np.arange(start = 0.00,stop = self.size_x ,step=0.04))) / self.size_x
        self.grad_const_y = (len(np.arange(start = 0.00,stop = self.size_y ,step=0.04))) / self.size_y
        self.g_noise_mag = 0.5

        self.metric = []
        self.pi_s = np.full([num_cf,num_cf],0.00)
        self.sigmas = np.full([num_cf,num_cf],0.00)
        self.r_x = np.full([1,num_cf],0.00)
        self.r_y = np.full([1,num_cf],0.00)
        self.f_x = np.full([1,num_cf],0.00)
        self.f_y = np.full([1,num_cf],0.00) 
        self.u = np.full([1,num_cf],0.00)   
        self.w = np.full([1,num_cf],0.00)   
        self.h = np.full([1,num_cf],0.00)   
        self.cf_x = np.full([1,num_cf],0.01)
        self.cf_y = np.full([1,num_cf],0.01)        
        self.cf_z = np.full([1,num_cf],0.01)

        self.vertices = []
        v1 = np.array([0.0,0.0])
        self.vertices.append(v1)
        v2 = np.array([6.5,0.0])
        self.vertices.append(v2)
        v3 = np.array([6.5,4.0])
        self.vertices.append(v3)
        v4 = np.array([0.0,4.0])
        self.vertices.append(v4)
        v5 = np.array([0.0,0.0])
        self.vertices.append(v5)
        self.vertices = np.asarray(self.vertices)       
        
        self.Dp = 2.0
        self.Dr = 0.5
        self.Da = 2.0   

        self.if_heading = float(if_heading)
        self.if_desired = float(if_desired)

        self.alpha = 1.0
        # self.alpha = 0.5
        self.gama = 1.0
        self.sigma_const = 0.4
        self.krep = 50.0
        self.L0 = 0.5
        self.umax_const = 0.15
        self.umin = 0.02

        self.wmax = 1.5708/2

        self.K1 = 0.3 #0.08

        self.K2 = 0.1
         #0.3

        self.epsilon = 12.0 
        self.pi = 3.1416
        self.repel_cos = np.cos(np.array([self.pi/2, self.pi, 3*self.pi/2, 0]))
        self.repel_sin = np.sin(np.array([self.pi/2, self.pi, 3*self.pi/2, 0]))

        for i in range(0,num_cf):
            self.h[0,i] = (6.2832) * np.random.rand()

        self.u = 0.0
        self.u_old = np.full([1,num_cf],0.0)
        self.w = 0.0
        self.dt = 0.05
        self.updated = 0.0
        self.sleepRate = 20

    def cf_pose_update(self):

        self.cf_x[0] = cf_positions[0]
        self.cf_y[0] = cf_positions[1]  
        # print(self.cf_x[0]) 

    def takeoff(self):

        traj = uav_trajectory.Trajectory()
        # for cf in range(num_cf):
        #     cfs[cf].setParam("kalman/robustTdoa",1)
        #     timeHelper.sleep(1.0)
        #     cfs[cf].setParam("kalman/resetEstimation",1)
        #     timeHelper.sleep(2.0)
            
        for cf in range(num_cf):
            # cfs[cf].takeoff(targetHeight=0.7, duration=3.0)
            traj.loadcsv("takeoff_traj.csv")
            cfs[cf].uploadTrajectory(0, 0, trajectory=traj)
            timeHelper.sleep(0.5)

        timeHelper.sleep(2.0)

        for cf in range(num_cf):
            cfs[cf].startTrajectory(0, timescale=1)
            timeHelper.sleep(0.5)

        timeHelper.sleep(traj.duration + 5.0)

    def land(self):

        time_diff = 0.0
        t0 = timeHelper.time()

        while time_diff<3.0:

            time_diff = timeHelper.time() - t0

            for cf in range(num_cf):
                cfs[cf].cmdVelocityWorld(np.array([0,0.0,-0.15]),yawRate=0)   

            timeHelper.sleepForRate(self.sleepRate) 

        # for cf in range(num_cf):
        #     cfs[cf].cmdStop()

    def calc_proximal(self):    
        self.xx1, self.xx2 = np.meshgrid(self.cf_x , self.cf_x)
        self.yy1, self.yy2 = np.meshgrid(self.cf_y , self.cf_y)
        self.d_ij_x = self.xx1 - self.xx2
        self.d_ij_y = self.yy1 - self.yy2
        self.d_ij = np.sqrt(np.multiply(self.d_ij_x,self.d_ij_x) + np.multiply(self.d_ij_y,self.d_ij_y))
        # if np.random.rand() > 0.5:
        #     print(min(self.d_ij[self.d_ij != 0]))
        self.ij_ang = np.arctan2(self.d_ij_y,self.d_ij_x)
        self.pi_s = deepcopy(self.d_ij)
        self.pi_s[self.pi_s > self.Dp] = 0.0

        if self.if_desired:
            for i in range(0, num_cf):  
                # self.sigmas[i,:] = np.full([1,num_cf],0.8) - np.multiply(np.power(np.divide(self.grad_vals[i], 255.00),1),(0.5))
                self.sigmas[i,:] = np.full([1,num_cf],0.5) + np.multiply(np.power(np.divide(self.grad_vals[i], 255.00),1),(0.3))
            self.pi_s[self.pi_s != 0] = -self.epsilon*(2*(np.divide( np.power(self.sigmas[self.pi_s != 0], 4), np.power(self.pi_s[self.pi_s != 0], 5))) - (np.divide( np.power(self.sigmas[self.pi_s != 0], 2), np.power(self.pi_s[self.pi_s != 0], 3))))
        else:
            self.pi_s[self.pi_s != 0] = -self.epsilon*(2*(np.divide( np.power(self.sigma_const, 4), np.power(self.pi_s[self.pi_s != 0], 5))) - (np.divide( np.power(self.sigma_const, 2), np.power(self.pi_s[self.pi_s != 0], 3))))

        self.px_s = np.multiply(self.pi_s, np.cos(self.ij_ang))
        self.py_s = np.multiply(self.pi_s, np.sin(self.ij_ang))

        self.pbar_xs = np.sum(self.px_s, axis=1)
        self.pbar_ys = np.sum(self.py_s, axis=1)        

    def calc_wall(self):
        boun_distances = np.full([num_cf,4],0.00)

        for i in range(0,4):
            p3_s = np.split(np.asarray(list(zip(self.cf_x[0],self.cf_y[0]))),num_cf)
            member = 0

            for p3 in p3_s:
                p3 = p3[0]
                num = abs((self.vertices[i+1][1]-self.vertices[i][1])*p3[0] - (self.vertices[i+1][0]-self.vertices[i][0])*p3[1] + self.vertices[i+1][0]*self.vertices[i][1] - self.vertices[i+1][1]*self.vertices[i][0])
                den = math.sqrt((self.vertices[i+1][1]-self.vertices[i][1])**2 + (self.vertices[i+1][0]-self.vertices[i][0])**2)
                boun_distances[member,i] = num/den
                member = member + 1

        for i in range(0,num_cf):
            nearwall_indeces = np.where(boun_distances[i,:]<self.Dr)
            repel_dir = [int(ii) for ii in nearwall_indeces[0]]
            repel_dist = np.full([1,len(nearwall_indeces[0])],0.00)
            for ii in range (0, len(nearwall_indeces[0])):
                repel_dist[0,ii] = boun_distances[i,repel_dir[ii]]

            if len(repel_dist) == 0:
                self.r_x[0,i] = 0
                self.r_y[0,i] = 0
            else:
                self.r_x[0,i] = 0
                self.r_y[0,i] = 0
                for ii in range(0, len(nearwall_indeces[0])):   
                    self.r_x[0,i] = self.r_x[0,i] + self.krep*np.multiply((1/repel_dist[0,ii] - 1/self.L0), np.divide(self.repel_cos[repel_dir[ii]], np.power(repel_dist[0,ii], 3)))
                    self.r_y[0,i] = self.r_x[0,i] + self.krep*np.multiply((1/repel_dist[0,ii] - 1/self.L0), np.divide(self.repel_sin[repel_dir[ii]], np.power(repel_dist[0,ii], 3)))  

    def calc_grad(self):
        self.g_noises = -self.g_noise_mag + 2*self.g_noise_mag*np.random.rand(1,num_cf)
        self.grad_x = np.ceil(np.multiply(self.cf_y[0],self.grad_const_x)) 
        self.grad_y = np.ceil(np.multiply(self.cf_x[0],self.grad_const_y)) 
        self.grad_y = self.grad_y.astype(int)
        self.grad_x = self.grad_x.astype(int)

        self.grad_x[self.grad_x >= 100] = 99
        self.grad_x[self.grad_x <= 0] = 0

        self.grad_y[self.grad_y >= 163] = 162
        self.grad_y[self.grad_y <= 0] = 0

        # print(self.g_noises[0])
        self.grad_vals = self.map[self.grad_x,self.grad_y] + self.g_noises[0]
        self.grad_vals[self.grad_vals<0]=0.0
        self.grad_vals[self.grad_vals>255]=255.0
        # print(self.grad_x[0],"X")
        # print(self.grad_y[0],"Y")
        # print(self.grad_vals[0],"G")     
        # print(self.cf_x[0],'Xpos')
        # print(self.cf_y[0],'Ypos')   
        self.metric = np.sum(self.grad_vals, axis=0)



    def calc_force(self):

        global headings

        self.f_x = self.alpha*self.pbar_xs + self.gama*self.r_x
        self.f_y = self.alpha*self.pbar_ys + self.gama*self.r_y 

        self.f_mag = np.sqrt(np.square(self.f_x) + np.square(self.f_y))
        self.glob_ang = np.arctan2(self.f_y, self.f_x)

        rels = self.glob_ang - self.h

        for hh in range(0,len(rels[0])):
            rels[0,hh] = self.wraptopi(rels[0,hh])

        self.u = self.K1 * np.multiply(self.f_mag , np.cos(rels)) + 0.05

        self.u[self.u>self.umax_const] = self.umax_const
        self.u[self.u<0] = 0


        self.w = self.K2 * np.multiply(self.f_mag , np.sin(rels))

        self.w[self.w>self.wmax] = self.wmax
        self.w[self.w<-self.wmax] = -self.wmax      

        for hh in range(0,len(self.h[0])):
            self.h[0,hh] = self.wraptopi(self.h[0,hh])  

        self.h[0] = self.h[0] + self.w[0]*self.dt 
        headings = self.h[0]  

    def wraptopi(self,x):
        x = x%(3.1415926*2)
        x = (x+(3.1415926*2))%(3.1415926*2)

        if (x>3.1415926):
            x = x - (3.1415926*2)

        return x    
              
    def publish_velocities(self):

        for j in range(0,num_cf):

            self.vel_cmd = np.array([self.u[0,j] * np.cos(self.h[0,j]),self.u[0,j] * np.sin(self.h[0,j]),0])

            self.u_old = deepcopy(self.u)

            cfs[j].cmdHover(vx=self.vel_cmd[0],vy=self.vel_cmd[1],yawRate=0,zDistance=0.7)

    def main_run(self): 
        bg = timeHelper.time()
        swarm.cf_pose_update()      
        swarm.calc_grad()
        swarm.calc_proximal()
        swarm.calc_wall()
        swarm.calc_force()  
        swarm.publish_velocities() 
        elaps = timeHelper.time() - bg
        # print(elaps)

    def signal_handler(self,sig, frame):

        print('You pressed Ctrl+C!')

        for i in range(num_cf):

            cfs[i].cmdStop()

        sys.exit(0)                   

if __name__ == '__main__':

    global cf_positions 

    realm = int(sys.argv[1]) #1-->SIM, 2-->REAL

    sleepRate = 20

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    server.settimeout(0.2)    

    num_cf = 5
    if_heading = 1
    if_desired = 1

    cfs = dict()
    Cswarm = Crazyswarm()
    feedback = pos_fb_updater() 
    timeHelper = Cswarm.timeHelper

    for i in range(0,num_cf):
        cfs[i] = Cswarm.allcfs.crazyflies[i]

    swarm = master_swarm(num_cf, if_heading, if_desired)
    signal(SIGINT, swarm.signal_handler)

    swarm.takeoff()

    flight_start = timeHelper.time()
    flight_elapsed = 0.0
    flight_duration = 270.0

    plot_modulo_counter = 0

    while flight_elapsed<flight_duration:

        flight_elapsed = timeHelper.time() - flight_start
        print(flight_elapsed)
        swarm.main_run()

        if realm == 1:
            for i in range(num_cf):
                cf_positions[0][i] = cfs[i].position()[0]
                cf_positions[1][i] = cfs[i].position()[1]
                cf_positions[2][i] = cfs[i].position()[2]
        else:
            cf_positions = feedback.get_positions()
            
        # print(cf_positions)

        #FOR UDP VISUALIZATION
        if (plot_modulo_counter%10) == 0:
            message = np.hstack((cf_positions[0], cf_positions[1], headings))
            message = message.tostring()
            server.sendto(message, ("localhost", 37020))
        ###

        plot_modulo_counter = plot_modulo_counter + 1
        timeHelper.sleepForRate(sleepRate)

    print("Finished!")

    swarm.land()

