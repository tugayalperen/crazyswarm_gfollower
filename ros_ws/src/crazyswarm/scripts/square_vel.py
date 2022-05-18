"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np
import uav_trajectory


TAKEOFF_DURATION = 3.5
edge_time = 5.0
sleepRate = 20



def main():

    time_diff = 0.0
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]


    # cf.setParam("kalman/robustTdoa",1)
    # cf.setParam("motion/disable",1)
    # print("robust tdoa enabled")    

    timeHelper.sleep(5.0)

    cf.takeoff(targetHeight=0.50, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    # traj = uav_trajectory.Trajectory()
    # traj.loadcsv("takeoff_traj.csv")
    # cf.uploadTrajectory(0, 0, trajectory=traj)
    #
    # cf.startTrajectory(0, timescale=1)
    # timeHelper.sleep(traj.duration)

    t0 = timeHelper.time()
    while time_diff<edge_time:
        time_diff = timeHelper.time() - t0
        cf.cmdHover(vx=0.15,vy=0.0,yawRate=0,zDistance=0.5)
        # cf.cmdVelocityWorld(np.array([0.15, 0.0, 0.0]), yawRate=0)
        timeHelper.sleepForRate(sleepRate)
        # print(cf.position())

    time_diff = 0.0
    t0 = timeHelper.time()
    while time_diff<edge_time:
        time_diff = timeHelper.time() - t0
        cf.cmdHover(vx=0.0,vy=0.15,yawRate=0,zDistance=0.5)
        # cf.cmdVelocityWorld(np.array([0, 0.15, 0.0]), yawRate=0)
        timeHelper.sleepForRate(sleepRate)
        # print(cf.getPos())
    
    time_diff = 0.0
    t0 = timeHelper.time()
    while time_diff<edge_time:
        time_diff = timeHelper.time() - t0
        cf.cmdHover(vx=-0.15,vy=0.0,yawRate=0,zDistance=0.5)
        # cf.cmdVelocityWorld(np.array([-0.15, 0.0, 0.0]), yawRate=0)
        timeHelper.sleepForRate(sleepRate)     
        # print(cf.getPos())
        
    time_diff = 0.0    
    t0 = timeHelper.time()
    while time_diff<edge_time:
        time_diff = timeHelper.time() - t0
        cf.cmdHover(vx=0.0,vy=-0.15,yawRate=0,zDistance=0.5)
        # cf.cmdVelocityWorld(np.array([0, -0.15, 0.0]), yawRate=0)
        timeHelper.sleepForRate(sleepRate)  
        # print(cf.getPos())                         

    time_diff = 0.0
    t0 = timeHelper.time()
    while time_diff<edge_time:
        time_diff = timeHelper.time() - t0
        cf.cmdVelocityWorld(np.array([0,0.0,-0.15]),yawRate=0)
        timeHelper.sleepForRate(sleepRate) 
        # print(cf.getPos())      

    cf.cmdStop()                   


if __name__ == "__main__":
    main()
