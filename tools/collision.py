#!/usr/bin/env python3
import cvxpy as cp
import numpy as np


def ellipsoid_collision_swept(E, p0, p1, q0, q1):
    """
    E = diag(r_x, r_y, r_z): major axis of ellipsoid
    p0 -> p1: line segment for ellipsoid 1
    q0 -> q1: line segment for ellipsoid 2

    solves a QP to check for a (swept) collision
    A collision occurs, if ||E^-1(p-q)||_2 <= 2
    see

    Wolfgang Hönig, James A. Preiss, T. K. Satish Kumar, Gaurav S. Sukhatme, Nora Ayanian:
    Trajectory Planning for Quadrotor Swarms. IEEE Trans. Robotics 34(4): 856-869 (2018)"""

    E_inverse = np.linalg.inv(E)

    # let first line segment be p0+alpha*(p1-p0), where alpha in [0,1]
    alpha = cp.Variable()
    # let second line segment be q0+beta*(q1-q0), where beta in [0,1]
    beta = cp.Variable()

    objective = cp.Minimize(cp.norm2(E_inverse @ ((p0+alpha*(p1-p0) - (q0+beta*(q1-q0))))))
    constraints = [0 <= alpha, alpha <=1, 0 <= beta, beta <= 1]
    prob = cp.Problem(objective, constraints)
    result = prob.solve()

    return result <= 2


def ellipsoid_collision_motion(E, p0, p1, q0, q1):
    """
    E = diag(r_x, r_y, r_z): major axis of ellipsoid
    p0 -> p1: line segment for ellipsoid 1
    q0 -> q1: line segment for ellipsoid 2

    solves a QP to check for a motion collision (i.e., assuming constant velocity movement)
    A collision occurs, if ||E^-1(p-q)||_2 <= 2
    see

    Wolfgang Hönig, James A. Preiss, T. K. Satish Kumar, Gaurav S. Sukhatme, Nora Ayanian:
    Trajectory Planning for Quadrotor Swarms. IEEE Trans. Robotics 34(4): 856-869 (2018)"""

    E_inverse = np.linalg.inv(E)

    # let first line segment be p0+alpha*(p1-p0), where alpha in [0,1]
    # let second line segment be q0+alpha*(q1-q0)
    alpha = cp.Variable()

    objective = cp.Minimize(
        cp.norm2(E_inverse @ ((p0+alpha*(p1-p0) - (q0+alpha*(q1-q0))))))
    constraints = [0 <= alpha, alpha <= 1]
    prob = cp.Problem(objective, constraints)
    result = prob.solve()

    return result <= 2

def main():
    E = np.diag([0.15, 0.15, 0.3])
    p0 = np.array([-3.5, -1, 2])
    p1 = np.array([-3.5, 3, 2])
    q0 = np.array([0.5, -1, 2])
    q1 = np.array([0.5, 3, 2])

    collides_swept = ellipsoid_collision_swept(E, p0, p1, q0, q1)
    collides_motion = ellipsoid_collision_motion(E, p0, p1, q0, q1)
    print(collides_swept, collides_motion)

if __name__ == "__main__":
    main()