#!/usr/bin/env python

import numpy as np

from ompl import base as ob
from ompl import control as oc
from ompl import app

#this method is called by ompl_app_custom.py
def configure(ss):
    # create a start state
    cspace = ss.getControlSpace()

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(0, 0.0)
    cbounds.setHigh(0, 3.0)

    cbounds.setLow(1, 0.0)
    cbounds.setHigh(1, 3.0)
    cspace.setBounds(cbounds)

    information = ss.getSpaceInformation()

    information.setStatePropagator(oc.StatePropagatorFn(propagate))


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

    state.setX(x)
    state.setY(y)
    state.setYaw(yaw)


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



def plan():
    """this function is used only when running the file as main file\
    it is not used if used as a plugin"""

    # construct the state space we are planning in
    ss = app.KinematicCarPlanning()

    ss.setEnvironmentMesh('./mesh/UniqueSolutionMaze_env.dae')
    ss.setRobotMesh('./mesh/car2_planar_robot.dae')
    # space = ob.SE2StateSpace()

    cspace = ss.getControlSpace()

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(0, -3.0)
    cbounds.setHigh(0, 3.0)

    cbounds.setLow(1, 0.0)
    cbounds.setHigh(1, 3.0)
    cspace.setBounds(cbounds)

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

    #si = ss.getSpaceInformation()

    information = ss.getSpaceInformation()

    information.setStatePropagator(oc.StatePropagatorFn(propagate))

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)


    # attempt to solve the problem
    solved = ss.solve(20.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())
        with open('path.txt', 'w') as outFile:
            outFile.write(ss.getSolutionPath().printAsMatrix())

def version():
    return 2

if __name__ == "__main__":
    plan()

print(__file__,'loaded')