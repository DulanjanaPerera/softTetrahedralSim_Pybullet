import pybullet as p
import time
import pybullet_data  # this apparently contains the plane and r2d2
import math
import numpy as np


# import threading


class TetraSim():
    def __init__(self, gui=False, force=100):
        # threading.Thread.__init__(self)
        self.gui = gui
        self.ur = None
        self.step_finished = False
        self.followcam = False
        self.maxForce = force
        self.zoom = 2
        self.dim_states = 14
        self.dim_actions = 8
        self.StartPos = [0, 0, 0.5]
        self.StartOrientation = p.getQuaternionFromEuler([math.pi, 0, 0])
        self.heading_dir = math.pi
        self.goal = [-4, 0, 0]
        self.prev_distance = 0.0

    def getPosition(self):
        if self.step_finished:
            return np.round(p.getLinkState(self.ur, 0)[0], 2)
        else:
            return np.array([0.0, 0.0, 0.0])

    def getFullPosition(self):
        if self.step_finished:
            xy1 = p.getLinkState(self.ur, 0)[0]
            xy2 = p.getLinkState(self.ur, 15)[0]
            xy3 = p.getLinkState(self.ur, 31)[0]
            xy4 = p.getLinkState(self.ur, 47)[0]

            return np.round([xy1[0], xy1[1], xy2[0], xy2[1], xy3[0], xy3[1], xy4[0], xy4[1]], 2)
        else:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def getRobotBasePosition(self):  # robot base cartesian location
        if self.step_finished:
            return np.round(p.getLinkState(self.ur, 0)[4], 2)
        else:
            return np.array([0.0, 0.0, 0.0])

    def getRobotBaseVelocity(self):  # robot base linear velocity
        if self.step_finished:
            return np.round(p.getBaseVelocity(self.ur)[0], 2)
        else:
            return np.array([0.0, 0.0, 0.0])

    def getRobotBaseOrientation(self):  # robot base orientation
        if self.step_finished:
            return np.round(p.getEulerFromQuaternion(p.getLinkState(self.ur, 0)[5]), 2)
        else:
            return np.array([0, 0, 0])

    def getHeadingError(self, curr_head):  # this function is mainly for pi heading direction
        if curr_head > 0:
            # return np.round(-(curr_head + des_head), 2)
            return np.round(-curr_head, 2)
        else:
            return np.round(curr_head, 2)

    def getGoalError(self):
        curr_pos = self.getRobotBasePosition()
        des_pos = self.goal
        mse = np.square(np.subtract(des_pos, curr_pos)).mean()
        return np.round(mse, 2)

    def getLegStates(self) -> list:
        joint_indices = np.arange(1, 67).tolist()
        joint_s = list(p.getJointStates(self.ur, joint_indices))
        joint_pos = []
        for s in joint_s:
            joint_pos.append(s[0])
        return joint_pos

    def getRewards(self, heading_error):
        # reward for keep the heading direction
        headErrorThreshold = abs(heading_error)
        if headErrorThreshold < 0.08:
            lambda_error = 2
        else:
            lambda_error = -1

        # reward for moving goal direction
        velocity = -self.getRobotBaseVelocity()[0]
        if velocity > 0.7:
            lambda_vel = 5
        else:
            lambda_vel = -2

        # Reward for distance traveled in the step
        distance = (self.prev_distance - self.getRobotBasePosition()[0])
        lambda_dis = distance * 10
        self.prev_distance = self.getRobotBasePosition()[0]

        # reward for reaching the goal
        goal_error = self.getGoalError()
        if abs(goal_error) < 0.5:
            lambda_goal = 100
        else:
            lambda_goal = -5

        return lambda_error + lambda_vel + lambda_dis + lambda_goal

    def resetEnv(self):
        p.resetBasePositionAndOrientation(self.ur, self.StartPos, self.StartOrientation)
        joint_list = np.arange(1, 67).tolist()
        for idx in joint_list:
            p.resetJointState(self.ur, idx, targetValue=0.0, targetVelocity=0.0)
        while (1):
            maxForce = np.ones(8) * self.maxForce
            [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir] = [0, 0, 0, 0, 0, 0, 0, 0]

            # position values
            positionVal = [math.cos(s_dir) * -s_str,
                           math.sin(s_dir) * -s_str,
                           math.cos(fr_dir) * -fr_str,
                           math.sin(fr_dir) * -fr_str,
                           math.cos(fl_dir) * -fl_str,
                           math.sin(fl_dir) * -fl_str,
                           math.cos(rr_dir) * -rr_str,
                           math.sin(rr_dir) * -rr_str]
            # print(self.ur)
            p.setJointMotorControlArray(self.ur, np.arange(1, 16, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[0], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(2, 17, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[1], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(18, 33, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[2], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(19, 34, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[3], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(35, 50, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[4], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(36, 51, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[5], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(51, 66, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[6], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(52, 67, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[7], forces=maxForce)

            if self.getRobotBasePosition()[2] == 0.2:
                break

    def getTopple(self):
        [ox, oy, oz] = self.getRobotBaseOrientation()
        if ox > 1.1 or oy < 0.2:
            return True
        else:
            return False

    def getTermination(self):
        if self.getGoalError() < 0.5:
            return True
        elif self.getTopple():
            return True
        else:
            return False

    def endSimulation(self):
        print("Ending Simulation...")
        p.disconnect()

    def run(self):

        if self.gui:
            print("Starting Simulation...")
            p.connect(p.GUI)
            self.followcam = True
        else:
            p.connect(p.DIRECT)
            self.followcam = False
        p.resetSimulation()
        if self.followcam:
            p.resetDebugVisualizerCamera(cameraDistance=self.zoom, cameraYaw=60, cameraPitch=-45,
                                         cameraTargetPosition=(0, 0, 1))
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        planeId = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -20)
        p.changeDynamics(planeId, -1, lateralFriction=1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.ur = p.loadURDF("./tetra.urdf", self.StartPos, self.StartOrientation, flags=p.URDF_USE_SELF_COLLISION)

    def step(self, action):
        """
        action is a list
        [back leg, right leg, lef leg, top leg]
        action = [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir]
        """
        self.step_finished = False
        maxForce = np.ones(8) * self.maxForce

        [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir] = action
        bending = [s_str, fr_str, fl_str, rr_str]
        [s_str, fr_str, fl_str, rr_str] = np.clip(bending, -math.pi / 8, math.pi / 8)

        positionVal = [math.cos(s_dir) * -s_str,
                       math.sin(s_dir) * -s_str,
                       math.cos(fr_dir) * -fr_str,
                       math.sin(fr_dir) * -fr_str,
                       math.cos(fl_dir) * -fl_str,
                       math.sin(fl_dir) * -fl_str,
                       math.cos(rr_dir) * -rr_str,
                       math.sin(rr_dir) * -rr_str]
        p.setJointMotorControlArray(self.ur, np.arange(1, 16, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[0], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(2, 17, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[1], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(18, 33, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[2], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(19, 34, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[3], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(35, 50, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[4], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(36, 51, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[5], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(51, 66, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[6], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(52, 67, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[7], forces=maxForce)

        p.stepSimulation()
        self.step_finished = True

        # calculate the states
        if self.step_finished:
            curr_dir = self.getRobotBaseOrientation()[2]
            heading_error = self.getHeadingError(curr_dir)
            goal_error = self.getGoalError()

            # states: heading direction, heading error, goal error, x-orientation, y-orientation, current position
            states = [np.round(self.heading_dir, 2),
                      heading_error,
                      goal_error,
                      self.getRobotBaseVelocity()[0],
                      self.getRobotBasePosition()[0]]
            reward = self.getRewards(heading_error)
            return self.getLegStates()

