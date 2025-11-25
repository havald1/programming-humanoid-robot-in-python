'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

from xmlrpc.server import SimpleXMLRPCServer
import threading
import numpy as np

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True,
                 rpc_port = 8000):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.rpc_server = SimpleXMLRPCServer(("localhost", rpc_port), allow_none=True)

        self.rpc_server.register_function(self.get_angle, "get_angle")
        self.rpc_server.register_function(self.set_angle, "set_angle")
        self.rpc_server.register_function(self.get_posture, "get_posture")
        self.rpc_server.register_function(self.execute_keyframes, "execute_keyframes")
        self.rpc_server.register_function(self.get_transform, "get_transform")
        self.rpc_server.register_function(self.set_transform, "set_transform")

        thread = threading.Thread(target=self.rpc_server.serve_forever, daemon=True)
        thread.start()

        print(f"Server läuft auf Port {rpc_port}")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, np.array(transform))

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

