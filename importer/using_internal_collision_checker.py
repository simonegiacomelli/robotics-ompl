# !/usr/bin/env python

# Author: Mark Moll

from math import sin, cos
from functools import partial
import numpy as np

try:
    from ompl import base as ob
    from ompl import control as oc
    from ompl import app
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import control as oc


class MyDecomposition(oc.GridDecomposition):
    def __init__(self, length, bounds):
        super(MyDecomposition, self).__init__(length, 2, bounds)

    def project(self, s, coord):
        coord[0] = s.getX()
        coord[1] = s.getY()

    def sampleFullState(self, sampler, coord, s):
        sampler.sampleUniform(s)
        s.setXY(coord[0], coord[1])


def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are
    # satisfied
    return spaceInformation.satisfiesBounds(state)


def mktr(x, y):
    return np.array([[1, 0, x],
                     [0, 1, y],
                     [0, 0, 1]])


def mkrot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])


def ddtr(vl, vr, l, dt):
    """ returns the pose transform for a motion with duration dt of a differential
    drive robot with wheel speeds vl and vr and wheelbase l """

    if (np.isclose(vl, vr)):  # we are moving straight, R is at the infinity and we handle this case separately
        return mktr((vr + vl) / 2 * dt, 0)  # note we translate along x ()

    omega = (vr - vl) / (2 * l)  # angular speed of the robot frame
    R = l * (vr + vl) / (vr - vl)

    # Make sure you understand this!
    return mktr(0, R) @ mkrot(omega * dt) @ mktr(0, -R)


def homogeneous(x, y, theta):
    return np.array([[np.cos(theta), -np.sin(theta), x],
                     [np.sin(theta), np.cos(theta), y],
                     [0, 0, 1]])


def propagate(start, control, duration, state):
    """ returns the pose transform for a motion with duration dt of a differential
    drive robot with wheel speeds vl and vr and wheelbase l """

    vr = control[0]
    vl = control[1]
    l = 1

    relative_pose = ddtr(vl, vr, l, duration)

    curr_pose = homogeneous(start.getX(), start.getY(), start.getYaw())

    pose = curr_pose @ relative_pose

    x = pose[0, 2]
    y = pose[1, 2]
    yaw = np.arctan2(pose[1, 0], pose[0, 0])

    # print("x = ", x)
    # print("y = ", y)
    # print("yaw = ", yaw)

    # print(control)
    # print("control[0] = ", control[0])
    # print("control[1] = ", control[1])

    # state.setX(start.getX() + x)
    # state.setY(start.getY() + y)
    # state.setYaw(start.getYaw() + yaw)

    state.setX(x)
    state.setY(y)
    state.setYaw(yaw)


def plan():
    # construct the state space we are planning in
    ss = app.SE2RigidBodyPlanning()

    ss.setEnvironmentMesh('./mesh/UniqueSolutionMaze_env.dae')
    ss.setRobotMesh('./mesh/car2_planar_robot.dae')
    # space = ob.SE2StateSpace()


    # create a start state
    start = ob.State(ss.getSpaceInformation())
    start().setX(3)
    start().setY(-3)
    start().setYaw(0.0)

    # create a goal state
    goal = ob.State(ss.getSpaceInformation())
    goal().setX(45)
    goal().setY(25)
    goal().setYaw(0.0)

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)


    # attempt to solve the problem
    solved = ss.solve(20.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())
        with open('path.txt', 'w') as outFile:
            outFile.write(ss.getSolutionPath().printAsMatrix())


if __name__ == "__main__":
    plan()
