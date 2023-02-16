from math import *
from config import *
from base import *

def kinematics(thetas):
    radPerDegree = pi / 180.0
    theta = [theta * radPerDegree for theta in thetas]
    if theta[1] < 0:
        theta[1] = -theta[1]
        AL = sqrt(FEMUR_LENGTH ** 2 + TIBIA_LENGTH ** 2 - 2 * FEMUR_LENGTH * TIBIA_LENGTH * cos(pi - theta[2]))
        beta = acos(
            (FEMUR_LENGTH ** 2 + AL ** 2 - TIBIA_LENGTH ** 2) / (2 * AL * FEMUR_LENGTH)
        )
        alpha2 = acos(
            (AL ** 2 + TIBIA_LENGTH ** 2 - FEMUR_LENGTH ** 2) / (2 * AL * TIBIA_LENGTH)
        )
        z = AL * sin(beta - theta[1])
        y_dot = AL * cos(beta - theta[1]) + COXA_LENGTH
        y = y_dot * cos(theta[0])
        x = y_dot * sin(theta[0])
    else:
        if theta[1] + theta[2] >= 90.0:
            theta_dot = 180.0 - (theta[1] + theta[2])
            y_dot = COXA_LENGTH + FEMUR_LENGTH * cos(theta[1]) - TIBIA_LENGTH * cos(theta_dot)
            y = y_dot * cos(theta[0])
            x = y_dot * sin(theta[0])
            z = FEMUR_LENGTH * sin(theta[1]) \
                + TIBIA_LENGTH * sin(theta_dot)
        else:
            y_dot = COXA_LENGTH + FEMUR_LENGTH * cos(theta[1]) \
                    + TIBIA_LENGTH * cos((theta[1] + theta[2]))
            y = y_dot * cos(theta[0])
            x = y_dot * sin(theta[0])
            z = FEMUR_LENGTH * sin(theta[1]) \
                + TIBIA_LENGTH * sin((theta[1] + theta[2]))
            beta = atan(z / (y_dot - COXA_LENGTH))
            alpha_1 = beta - theta[1]
            alpha_2 = theta[2] - alpha_1
    return point3d(x, y, z)

def Inversekinematics(Coordinate: point3d):
    degreePerRad = 180.0 / pi
    x, y, z = Coordinate.x, Coordinate.y, Coordinate.z
    try:
        theta1 = atan(x / y)
        y_dot = y / cos(theta1)
        AL = sqrt((y_dot - COXA_LENGTH) ** 2 + z ** 2)
        alpha1 = acos(
            (FEMUR_LENGTH ** 2 + AL ** 2 - TIBIA_LENGTH ** 2) / (2 * FEMUR_LENGTH * AL)
        )
        alpha2 = acos(
            (TIBIA_LENGTH ** 2 + AL ** 2 - FEMUR_LENGTH ** 2) / (2 * TIBIA_LENGTH * AL)
        )
        beta = atan(z / (y_dot - COXA_LENGTH))
        theta1 = theta1 * degreePerRad
        theta2 = (beta - alpha1) * degreePerRad
        theta3 = (alpha2 + alpha1) * degreePerRad
        return [theta1, theta2, theta3]
    except:
        try:
            theta1 = atan(x / y)
            y_dot = y / cos(theta1)
            AL = sqrt((y_dot - COXA_LENGTH) ** 2 + z ** 2)
            alpha1 = atan(z / (y_dot - COXA_LENGTH))
            beta = acos(
                FEMUR_LENGTH**2 + AL**2 - TIBIA_LENGTH**2
                / (2 * AL * FEMUR_LENGTH)
            )
            alpha2 = acos(
                AL**2 + TIBIA_LENGTH**2 - FEMUR_LENGTH**2
                / (2 * AL * TIBIA_LENGTH)
            )
            theta2 = beta - alpha1
            theta3 = beta + alpha2
            return [theta1 * degreePerRad, -theta2 * degreePerRad, theta3 * degreePerRad]
        except:
            raise ValueError


if __name__ == '__main__':
    theta = [45, 15.0, 110.0]
    co = kinematics(theta)



