import numpy as np
import node

# helper functions

def updatePosition(prev, measured, Kn):

    # filtering eqution: x_n,n = x_n,n-1 + Kn(z_n - x_n,n-1)
    curr_x = prev[0] + Kn[0]*( measured[0] - prev[0] )
    curr_y = prev[1] + Kn[1]*( measured[1] - prev[1] )
    curr_z = prev[2] + Kn[2]*( measured[2] - prev[2] )

    return (curr_x, curr_y, curr_z)

def updateCovariance(prev_variance, Kn):
    var_x = (1 - Kn[0]) * prev_variance[0]
    var_y = (1 - Kn[1]) * prev_variance[1]
    var_z = (1 - Kn[2]) * prev_variance[2]

    return (var_x, var_y, var_z)

def updateGain(prev_variance, measurement_variance):
    Kn_x = prev_variance[0] / ( prev_variance[0] + measurement_variance[0] )
    Kn_y = prev_variance[1] / ( prev_variance[1] + measurement_variance[1] )
    Kn_z = prev_variance[2] / ( prev_variance[2] + measurement_variance[2] )

    return (Kn_x, Kn_y, Kn_z)

def stateExtrapolation(position, dt, velocity):
    next_x = position[0] + dt*velocity[0]
    next_y = position[1] + dt*velocity[1]
    next_z = position[2] + dt*velocity[2]

    return (next_x, next_y, next_z)

def selectNode(prevFrameNodes, pos_xyz, t, range):
    x = pos_xyz [0]
    y = pos_xyz [1]
    z = pos_xyz [2]
    # weight each node within search area by proximity to pos_xyz
    nodeWeights = []
    maxWeightIndex = -1
    currIndex = 0
    for node in prevFrameNodes:
        xDiff = x - node.nx
        yDiff = y - node.ny
        zDiff = z - node.nz

        if xDiff < range and yDiff < range and zDiff < range:
            if maxWeightIndex == -1: # if any node within range, update this value
                maxWeightIndex = 0
            weight = np.sqrt( (xDiff)**2 + (yDiff)**2 + (zDiff)**2 )
            nodeWeights.append(weight)
            if nodeWeights[maxWeightIndex] < weight:
                maxWeightIndex = currIndex
        else:
            nodeWeights.append(0)

        currIndex += 1

    if maxWeightIndex == -1:
        return None
    
    # select a node
    return prevFrameNodes[maxWeightIndex]

def initNode(node):
    if node.x != None:
        return None
    
    node.x = node.mx
    node.y = node.my
    node.z = node.mz

    node.varx = 0.1
    node.vary = 0.1
    node.varz = 0.1




# Primary function:

def kalmanFilter(node, prevFrameNodes):
    # measurement uncertainty
    mvar_xyz = (0.1, 0.1, 0.1) # measurement variance for x, y, and z directions [meters]
    # velocity uncertainty [m/s]?
    vvar = 0.3
    # range in meters to form search cube
    searchArea = 1 

    # node init (check if within previous frame)
    node = initNode(node)
    # init position estimate
    curr_xyz = (node.x, node.y, node.z)
    # init estimate uncertainty
    pvar_xyz = (0.1, 0.1, 0.1) 
    # init velocity estimate (no movement)
    node.vx = 0
    node.vy = 0
    node.vz = 0
    # init dt (updated if previous node exists)
    dt = 0
    # init kalman gain
    Kn_xyz = updateGain(pvar_xyz, mvar_xyz)
    node.kx = Kn_xyz[0]
    node.ky = Kn_xyz[1]
    node.kz = Kn_xyz[2]
    # init estimate uncertainty
    var_xyz = updateCovariance(pvar_xyz, Kn_xyz)
    node.varx = var_xyz[0]
    node.vary = var_xyz[1]
    node.varz = var_xyz[2]

    # Weighted Selector
    prevNode = selectNode(prevFrameNodes, curr_xyz, node.t, searchArea)

    # State Update, skip initial node
    if prevNode != None:
        # extract previous node values
        pvar_xyz = (prevNode.varx, prevNode.vary, prevNode.varz)
        prev_xyz = (prevNode.x, prevNode.y, prevNode.z)
        
        # update kalman gain
        Kn_xyz = updateGain(pvar_xyz, mvar_xyz)
        # update estimate uncertainty
        var_xyz = updateCovariance(pvar_xyz, Kn_xyz)
        # update position estimate
        curr_xyz = updatePosition(prev_xyz, node.m_pos, Kn_xyz)

        dt = node.t - prevNode.t

        # update node data structure
        node.id = prevNode.id

        node.varx = var_xyz[0]
        node.vary = var_xyz[1]
        node.varz = var_xyz[2]

        node.x = curr_xyz[0]
        node.y = curr_xyz[1]
        node.z = curr_xyz[2]

        node.vx = node.x - prevNode.x / dt
        node.vy = node.y - prevNode.y / dt
        node.vz = node.z - prevNode.z / dt

    # Predict next state
    (next_x, next_y, next_z) = stateExtrapolation((node.x, node.y, node.z), dt, (node.vx, node.vy, node.vz))
    node.nx = next_x
    node.ny = next_y
    node.nz = next_z
    # extrapolate covariance
    doot = dt**2 * vvar
    node.varx = node.varx + doot
    node.vary = node.vary + doot
    node.varz = node.varz + doot

    return node
    
    
    

        

