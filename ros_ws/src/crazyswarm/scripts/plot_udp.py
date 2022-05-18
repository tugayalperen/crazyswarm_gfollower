#!/usr/bin/env python3

import matplotlib

import socket
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import cv2

num_cf = 2
size_x = 6.5
size_y = 4.0

harita = sio.loadmat('/home/tugay/Environments/circle_4x65.mat')
# size_x = 23
# size_y = 8
#
# harita = sio.loadmat('/home/tugay/Environments/circle_8x23.mat')
harita = harita['I']
# harita = np.full([100,163],125.0)

# harita =  cv2.imread("/Users/tugaykaraguzel/Desktop/CI_Documents/Master Thesis/Energy Maps/circle.PNG")
# harita = cv2.cvtColor(harita, cv2.COLOR_BGR2GRAY)
# harita = cv2.flip(harita, -1)
# grad_const = (len(np.arange(start = 0.00,stop = 10.00 ,step=0.0250/2))) / 10.00

# self.grad_y = np.ceil(np.multiply(self.cf_x[0],self.grad_const)) 
# self.grad_x = np.ceil(np.multiply(self.cf_y[0],self.grad_const)) 
# self.grad_y = self.grad_y.astype(int)
# self.grad_x = self.grad_x.astype(int)
# self.grad_vals = self.map[self.grad_x,self.grad_y]
# self.metric = np.sum(self.grad_vals, axis=0)

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP

# Enable port reusage so we will be able to run multiple clients and servers on single (host, port).
# Do not use socket.SO_REUSEADDR except you using linux(kernel<3.9): goto https://stackoverflow.com/questions/14388706/how-do-so-reuseaddr-and-so-reuseport-differ for more information.
# For linux hosts all sockets that want to share the same address and port combination must belong to processes that share the same effective user ID!
# So, on linux(kernel>=3.9) you have to run multiple servers and clients under one user to share the same (host, port).
# Thanks to @stevenreddie
# client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

# Enable broadcasting mode
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

client.bind(("", 37020))

plt.ion()
plt.show()
log_x = np.full([1,num_cf],0.00)
log_y = np.full([1,num_cf],0.00)
log_h = np.full([1,num_cf],0.00)

try:
    while True:
        # Thanks @seym45 for a fix
        data, addr = client.recvfrom(1024)
        data_array = np.fromstring(data,dtype=np.float64)
        data_array = data_array.reshape((3,num_cf))

        plt.imshow(harita, extent=[0, size_x, 0, size_y])

        # print("received message: ", data_array )
        
        plt.axis([0,size_x,0,size_y])
        plt.scatter(data_array[0],data_array[1],c='#ff7f0e')
        plt.quiver(data_array[0], data_array[1], np.cos(data_array[2]), np.sin(data_array[2]))
        
        log_x = np.vstack([log_x,data_array[0]])
        log_y = np.vstack([log_y,data_array[1]])
        log_h = np.vstack([log_h,data_array[2]])

        plt.draw()
        plt.pause(0.000001)
        plt.clf()

except KeyboardInterrupt:
    print("Press Ctrl-C to terminate while statement")
    sio.savemat('/home/tugay/Results/log_x.mat', {'x':log_x})
    sio.savemat('/home/tugay/Results/log_y.mat', {'y':log_y})
    sio.savemat('/home/tugay/Results/log_h.mat', {'h':log_h})
    pass


        