'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from autograd.numpy import identity
import autograd.numpy as np
from autograd import grad
#from scipy.linalg import pinv
from scipy.spatial.transform import Rotation

class InverseKinematicsAgent(ForwardKinematicsAgent):

    def Effector (self, effector_name, winkel):
        joints = self.chains[effector_name]
        angle_dict = dict(self.perception.joint)
        for name, angle in zip(joints, winkel):
            angle_dict[name] = float(angle)

        self.forward_kinematics(angle_dict)
        last_joint = joints[-1]
        T = np.asarray(self.transforms[last_joint], dtype=float)
        pos = T[0:3, 3]
        pos = np.asarray(pos, dtype=float).reshape(3,)
        return pos

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        max_iteration = 1000
        alpha = 0.1
        eps, toleranz = 1e-3, 1e-3
        
        joints = self.chains[effector_name]
        n = len(joints)

        target_pos = transform[0:3, 3].astype(float)

        joint_angle_ak = np.array([self.perception.joint[i] for i in joints], dtype = float)

        # def error (joint_angles):
        #     tmp = self.Effector(effector_name, joint_angles)
        #     e = tmp - target_pos
        #     return 0.5 * np.dot(e,e)
        
        #grad_E = grad(error)
        joint_angles = joint_angle_ak

        for _ in range(max_iteration):
            p = self.Effector(effector_name, joint_angles)
            e = p - target_pos
            error_norm = np.linalg.norm(e)
            if error_norm < toleranz:
                break

            E0 = float(0.5 * np.dot(e,e))

            g  = np.zeros_like(joint_angles)

            for i in range(len(joint_angles)):
                q_eps = joint_angles.copy()
                q_eps[i] += eps
                p_eps = self.Effector(effector_name, q_eps)
                e_eps = p_eps - target_pos
                E_eps = float(0.5 * np.dot(e_eps, e_eps))
                g[i] = (E_eps - E0) / eps
            
            # g_norm = np.linalg.norm(g)
            # if g_norm < toleranz:
            #     break
            joint_angles = joint_angles - alpha * g


        p_final = self.Effector(effector_name, joint_angles)
        error_vec = p_final - target_pos
        print("IK check for", effector_name)
        print("  target position :", target_pos)
        print("  reached position:", p_final)
        print("  error norm      :", np.linalg.norm(error_vec))
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

        time = 3.0
        joints = self.chains[effector_name]
        current_angle = dict(self.perception.joint)

        joint_angle_target = self.inverse_kinematics(effector_name, transform)

        names = []
        times = []
        keys = []

        for joint_name, target_angle in zip(joints, joint_angle_target):
            first_angle = float(current_angle[joint_name])
            names.append(joint_name)
            times.append([1.0, 1.0 + time])
            keys.append([
                [first_angle, [3.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
                [float(target_angle), [3.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
            ])

        self.keyframes = (names, times, keys)

        

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, 3] = 0.05
    T[2, 3] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

# if __name__ == '__main__':
#     agent = InverseKinematicsAgent()
#     q_test = np.array([0.0, 0.3, -0.8, 1.5, -0.7, -0.3])

#     angle_dict = dict(agent.perception.joint)
#     for name, angle in zip(agent.chains['LLeg'], q_test):
#         angle_dict[name] = float(angle)

#     agent.forward_kinematics(angle_dict)
#     last_joint = agent.chains['LLeg'][-1]
#     T = np.asarray(agent.transforms[last_joint], dtype=float)

#     agent.set_transforms('LLeg', T)
#     agent.run()
