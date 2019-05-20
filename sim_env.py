from vrep_env_base import VrepEnvBase
import vrep as vrep
import numpy as np
from future.utils import viewitems
import time

class SimEnv(VrepEnvBase):

    '''
    'port_num' determines which simulation this env is connected to
    'suffix' determines which robot this env is connected to
    'reset' is a class that should return a dict with new_joint_angles as an entry
    '''
    def __init__(self, config={}, port_num=19997):

        super(SimEnv, self).__init__(config, port_num)

        #### quad handle ####
        _, self.quad_handle = vrep.simxGetObjectHandle(self.clientID, "Quadricopter", vrep.simx_opmode_blocking)
            
        #### quad target handle ####
        _, self.quad_target_handle = vrep.simxGetObjectHandle(self.clientID, "Quadricopter_target", vrep.simx_opmode_blocking)

        #### moving obstacle handle ####
        _, self.moving_obstacle_handle = vrep.simxGetObjectHandle(self.clientID, "moving_obstacle", vrep.simx_opmode_blocking)

        #### obstacle handles ####
        self.obstacle_handles = []
        self.obs_name = ["obstacle1", "obstacle2", "obstacle3", "obstacle4","obstacle5"]
        for obs_name in self.obs_name:
            _, obs_h = vrep.simxGetObjectHandle(self.clientID, obs_name, vrep.simx_opmode_blocking)
            self.obstacle_handles.append(obs_h)
            
    def set_quad_goal_position(self, pt):
        handle = self.quad_target_handle
        pos = pt
        vrep.simxSetObjectPosition(self.clientID, handle, -1, pos, vrep.simx_opmode_oneshot)
        self.synchronous_trigger()

        
    def get_quad_pose(self):
        handle = self.quad_handle
        rct = 1
        pos = np.zeros(3)
        while rct != 0 or np.linalg.norm(pos) < 0.001:    
            rct, pos = vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_streaming)
            rcq, quat = vrep.simxGetObjectQuaternion(self.clientID, handle, -1, vrep.simx_opmode_streaming)
            
        return np.array(pos), np.array(quat)

    def get_moving_obstacle_pose(self):
        handle = self.moving_obstacle_handle
        rct = 1
        pos = np.zeros(3)
        while rct != 0 or np.linalg.norm(pos) < 0.001:    
            rct, pos = vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_streaming)
            rcq, quat = vrep.simxGetObjectQuaternion(self.clientID, handle, -1, vrep.simx_opmode_streaming)
            
        return np.array(pos), np.array(quat)

        
    def get_obstacle_info(self):
        obs_info = []
        for obs_name, obs_h in zip(self.obs_name, self.obstacle_handles):
            rc = 1
            while rc != 0:
                rc, pos = vrep.simxGetObjectPosition(self.clientID, obs_h, -1, vrep.simx_opmode_streaming)
            obs_info.append({'name': obs_name, 'position': pos, 'radius': 1., 'height': 2.})

        return obs_info

    def synchronous_trigger(self):
        vrep.simxSynchronousTrigger(self.clientID)        
   
if __name__ == "__main__":
    sim_env = SimEnv()
    while True:
        print("quad pose:", sim_env.get_quad_pose())
        print("moving obstacle pose:", sim_env.get_moving_obstacle_pose())
        print("obstacle pose:", sim_env.get_obstacle_info())
        sim_env.set_quad_goal_position([-0.45, 1.85, 0.5])
        sim_env.synchronous_trigger()
        time.sleep(0.02)