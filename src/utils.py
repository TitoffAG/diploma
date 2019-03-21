import numpy as np
from math import radians, sin, cos
from pyquaternion import Quaternion

def transformEulerAnglesToComponentsQuaternion(teta, psi, phi):
    teta_radians = radians(teta)
    psi_radians = radians(psi)
    phi_radians = radians(phi)

    lambda0 = cos(teta/2) * cos((psi + phi)/2)
    lambda1 = sin(teta/2) * cos((psi - phi)/2)
    lambda2 = sin(teta/2) * sin((psi - phi)/2)
    lambda3 = cos(teta/2) * sin((psi + phi)/2)

    lambdaQ = Quaternion(lambda0, lambda1, lambda2, lambda3).unit
    print('lambda = ', lambdaQ)
    print('norm of lambda = ', lambdaQ.norm)

    lambda0, lambda1, lambda2, lambda3 = lambdaQ

    return lambda0, lambda1, lambda2, lambda3


def residualСalculation(lambdaT_pribl, lambdaT):
    lambdaT_priblQ = Quaternion(lambdaT_pribl)
    lambdaTQ = Quaternion(lambdaT)
    return (lambdaT_priblQ.conjugate * lambdaTQ).vector
