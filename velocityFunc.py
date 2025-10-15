import numpy as np

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