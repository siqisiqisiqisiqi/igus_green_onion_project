import numpy as np

def angle_thres(yaw):
    while yaw > 295:
        yaw = yaw - 360
    while yaw < -65:
        yaw = yaw + 360
    return yaw