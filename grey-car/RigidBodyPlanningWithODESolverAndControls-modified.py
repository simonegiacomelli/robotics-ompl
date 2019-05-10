#!/usr/bin/env python


# Author: Mark Moll

from math import sin, cos, tan
from functools import partial

try:
    from ompl import base as ob
    from ompl import control as oc
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import control as oc


def kinematicCarODE(q, u, qdot):
    # const double velocity = u[0];
    # const double steeringAngle = u[1];

    theta = q[2]
    carLength = 0.2
    velocity = u[0]
    qdot[0] = velocity * cos(theta)
    qdot[1] = velocity * sin(theta)
    steeringAngle = u[1]
    qdot[2] = velocity * tan(steeringAngle) / carLength


def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are
    # satisfied
    return spaceInformation.satisfiesBounds(state)


def plan():
    # construct the state space we are planning in
    # The state space of the car is SE(2) (x and y position with one angle for orientation).
    space = ob.SE2StateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    # ?? for this simple car model consists of the velocity and steering angle, both real valued.
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-.3)
    cbounds.setHigh(.3)
    cspace.setBounds(cbounds)

    # define a simple setup class
    global ss
    ss = oc.SimpleSetup(cspace)
    validityChecker = ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation()))
    ss.setStateValidityChecker(validityChecker)
    ode = oc.ODE(kinematicCarODE)
    #odeSolver = oc.ODEBasicSolver(ss.getSpaceInformation(), ode)
    odeSolver = oc.ODEErrorSolver(ss.getSpaceInformation(), ode)
    propagator = oc.ODESolver.getStatePropagator(odeSolver)
    ss.setStatePropagator(propagator)

    # create a start state
    start = ob.State(space)
    start().setX(-0.5)
    start().setY(0.0)
    start().setYaw(0.0)

    # create a goal state
    goal = ob.State(space)
    goal().setX(0.0)
    goal().setY(0.5)
    goal().setYaw(0.0)

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # attempt to solve the problem
    solved = ss.solve(120.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())


if __name__ == "__main__":
    plan()
