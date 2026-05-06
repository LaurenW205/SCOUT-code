# node struct definition
class Node:
    
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