# node struct definition
class Node:
    # fully defined constructor
    def __init__(self, id, measured_pos, t, pos, variance, vel, n_pos, gain):

        # id for tracking purposes
        self.id = id
        
        self.mx = measured_pos[0] 
        self.my = measured_pos[1]
        self.mz = measured_pos[2]
        self.m_pos = measured_pos

        self.t = t

        self.x = pos[0] 
        self.y = pos[1]
        self.z = pos[2]

        self.varx = variance[0] 
        self.vary = variance[1]
        self.varz = variance[2]

        self.vx = vel[0] 
        self.vy = vel[1]
        self.vz = vel[2]

        self.nx = n_pos[0]
        self.ny = n_pos[1]
        self.nz = n_pos[2]

        self.kx = gain[0]
        self.ky = gain[1]
        self.kz = gain[2]

    # live sample constructor
    def __init__(self, id, measured_pos, t):
        
        self.id = id

        self.mx = measured_pos[0] 
        self.my = measured_pos[1]
        self.mz = measured_pos[2]
        self.m_pos = measured_pos

        self.t = t

        self.x = None
        self.y = None
        self.z = None

        self.varx = None
        self.vary = None
        self.varz = None

        self.vx = None
        self.vy = None
        self.vz = None

        self.nx = None
        self.ny = None
        self.nz = None

        self.kx = None
        self.ky = None
        self.kz = None
