'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezier_interpolation(self, i, p0, p1, p2, p3):
        return (((1-i)**3) * p0) + (3 * ((1- i)**2) * i * p1)+ (3 * (1- i) * (i**2) * p2) + ((i**3) * p3)  

    def control_points(self, a0, a1, next_hendl, prev_hendl):
        offset_output = (next_hendl[2] if next_hendl is not None else 0.0)
        offset_prev = (prev_hendl[2] if prev_hendl is not None else 0.0)
        p0 = a0 # startwinkel
        p1 = a0 + float(offset_output) # verschobener winkel
        p2 = a1 + float(offset_prev)
        p3 = a1 # outputwinkel
        return p0, p1, p2, p3

    def find_segment(self, time_joint_list, time_now):
            for i in range(len(time_joint_list) - 1):
                if time_joint_list[i] <= time_now < time_joint_list[i+1]:
                    return i
            return max(0, len(time_joint_list)-2)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes

        if not names:
            return target_joints
        if self.start_time is None:
            self.start_time = perception.time
        relative_time = perception.time - self.start_time

        for i, joint_name in enumerate(names):
            key_list = keys[i]
            time_list = times[i]

            if not time_list:
                continue

            if relative_time <= time_list[0]: #vor erstem Keyframe
                target_joints[joint_name] = key_list[0][0]
                continue
            if relative_time >= time_list[-1]: #nach letztem keyframe
                target_joints[joint_name] = key_list[-1][0]
                continue
            #find segment
            tmp = self.find_segment(time_list, relative_time)
            start, end = time_list[tmp], time_list[tmp+1]
            normalisied_time = (relative_time - start) / (end - start) if (end > start) else 0.0

            #Hendlers + KEyframes
            a0, prev_hendle_0, next_hendler_0 = key_list[tmp]
            a1, prev_hendle_1, next_hendler_1 = key_list[tmp + 1]

            #Kontrollpunkte
            p0, p1, p2, p3 = self.control_points(a0, a1, next_hendler_0, prev_hendle_1)

            Bezier = self.bezier_interpolation(normalisied_time, p0, p1, p2, p3)

            target_joints[joint_name] = Bezier

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
