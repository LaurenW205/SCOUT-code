import numpy as np
import math

def xyVelocity(nodes):

    vx = []
    vy = []
    
    prev = None
    for node in nodes:
        if prev == None:
            prev = node
            continue
        
        dx = node.x - prev.x
        dy = node.y - prev.y
        dt = node.t - prev.t

        prev = node
        vx.append(dx/dt)
        vy.append(dy/dt)

    vx = np.array(vx)
    vy = np.array(vy)

    return vx, vy


def vthetaVelocity(nodes):

    v = []
    theta = []

    prev = None
    for node in nodes:
        if prev == None:
            prev = node
            continue

        dx = node.x - prev.x
        dy = node.y - prev.y
        dt = node.t - prev.t

        vel = math.sqrt((dx/dt)**2 + (dy/dt)**2) # sqrt(x^2 + y^2)
        rad = None
        if dx == 0: # edge case to prevent divide by 0 error
            rad = math.pi / 2
        else:
            rad = math.atan(dy/dx)
        deg = rad*180/math.pi

        prev = node
        v.append(vel)
        theta.append(deg)

    v = np.array(v)
    theta = np.array(theta)

    return v, theta
