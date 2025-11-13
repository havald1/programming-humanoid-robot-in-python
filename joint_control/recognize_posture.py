'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
#from keyframes import hello
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead
import pickle
import numpy as np
import os


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        base_dir = os.path.dirname(__file__)
        path = os.path.join(base_dir, "robot_pose.pkl")

        #self.posture_classifier = None  # LOAD YOUR CLASSIFIER
        with open(path, "rb") as f:
            self.posture_classifier = pickle.load(f)
        self.class_names = ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']


        #print("Typ:", type(self.posture_classifier))


    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        joint = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        try:
            joint_val = [perception.joint[name] for name in joint]
        except Exception:
            return posture
        
        Angle_X, Angle_Y = perception.imu
        feature = joint_val + [Angle_X, Angle_Y]
        tmp = np.array(feature, dtype=float).reshape(1, -1)
        
        lable = int(self.posture_classifier.predict(tmp)[0])
        if 0 <= lable < len(self.class_names):
            posture = self.class_names[lable]
        else:
            posture = 'unknown'

        #print("Predicted label:", lable, "posture:", posture)
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
