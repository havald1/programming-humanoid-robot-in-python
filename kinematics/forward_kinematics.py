'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import autograd.numpy as np
from numpy.matlib import matrix, identity, cos, sin

from recognize_posture import PostureRecognitionAgent

JOINT_VAL = {
            "HeadYaw" : (( 0.0, 0.0, 0.1265), "z"),
            "HeadPitch" : ((0.0, 0.0, 0.0), "y"),
            "LShoulderPitch" : ((0.0, 0.098, 0.1), "y"),
            "LShoulderRoll" : ((0.0, 0.0, 0.0), "z"),
            "LElbowYaw" : ((0.105, 0.015, 0.0), "x"),
            "LElbowRoll" : ((0.0, 0.0, 0.0), "z"),
            "LHipYawPitch" : ((0.0, 0.05, -0.085), "y"),
            "LHipRoll" : ((0.0, 0.0, 0.0), "x"),
            "LHipPitch" : ((0.0, 0.0, 0.0), "y"),
            "LKneePitch" : ((0.0, 0.0, -0.1), "y"),
            "LAnklePitch" : ((0.0, 0.0, -0.1029), "y"),
            "LAnkleRoll" : ((0.0, 0.0, 0.0), "x"),
            "RShoulderPitch" : ((0.0, 0.098, 0.1), "y"),
            "RShoulderRoll" : ((0.0, 0.0, 0.0), "z"),
            "RElbowYaw" : ((0.105, 0.015, 0.0), "x"),
            "RElbowRoll" : ((0.0, 0.0, 0.0), "z"),
            "RHipYawPitch" : ((0.0, 0.05, -0.085), "y"),
            "RHipRoll" : ((0.0, 0.0, 0.0), "x"),
            "RHipPitch" : ((0.0, 0.0, 0.0), "y"),
            "RKneePitch" : ((0.0, 0.0, -0.1), "y"),
            "RAnklePitch" : ((0.0, 0.0, -0.1029), "y"),
            "RAnkleRoll" : ((0.0, 0.0, 0.0), "x"),
        }

class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       "LArm": ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"],
                       "LLeg": ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"],
                       "RLeg": ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"],
                       "RArm": ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
                       }
        

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def Rx (self, theta):
        c, s = cos(theta), sin(theta)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1],
        ], dtype = float)
    
    def Ry (self, theta):
        c, s = cos(theta), sin(theta)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1],
        ], dtype = float)
    
    def Rz (self, theta):
        c, s = cos(theta), sin(theta)
        return np.array([
            [c, s, 0, 0],
            [-s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype = float)
    
    def translate (self, x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ], dtype = float)
        

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        val = JOINT_VAL.get(joint_name)

        (x, y, z), axe = val
        rotate = {"x": self.Rx, "y": self.Ry, "z": self.Rz}
        R = rotate[axe](joint_angle)
        T = self.translate(x,y,z,) @ R
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl # matrix multiplication
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
