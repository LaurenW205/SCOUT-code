## class Node(x, y, t, frame_num)
#
# @initParam x - integer denoting x coordinate (in pixels)
# @initParam y - integer denoting y coordinate (in pixels)
# @initParam x - double denoting time at (x,y) position (in seconds)
# @initParam frame_num - frame number/index since data recording began
#
# @Param id - unique identifier to track Nodes across frames
# @Param dxdf - Node velocity in x pixels per frame
# @Param dydf - Node velocity in y pixels per frame
# @Param search_r - search radius (square) for region of interest
#
# @Function self.trackID(prevNodes, currIDCount, frameDim)
#   param prevNodes - array of Nodes to look for in current frame ([] if first iteration)
#   param currIDCount - current number of unique objects tracked
#   param frameDim - tuple containing frame dimensions, form: (width, height) 
#   returns: Current ID count (number of tracked objects) for main function variable
#
# @Function Node.getPrevNodes(allNodes)
#   param allNodes - array of all recorded Nodes
#   returns: array of Nodes from previous frame (Nodes to check for in current frame)
#
# @Function Node.getNodeByID(allNodes, id)
#   param allNodes - array of all recorded Nodes
#   param id - Node id to extract from allNodes
#   returns: array of Nodes with matching id


class Node:
    def __init__(self, x, y, t, frame_num):
        
        self.x = x
        self.y = y
        self.t = t
        self.frame_num = frame_num

        # set by calling trackID() function
        self.id = 0
        self.dxdf = 0
        self.dydf = 0
        self.search_r = 0
        
    def trackID(self, prevNodes, currIDCount, frameDim):

        # extract frame dimensions and split into thirds
        frame_W = frameDim[0]
        sect_1x = frame_W//3
        sect_2x = 2*frame_W//3
        frame_H = frameDim[1]
        sect_1y = frame_H//3
        sect_2y = 2*frame_H//3

        # excludes first Node ever to prevent array segmentation fault
        if currIDCount > 0:
            for prev in prevNodes:

                # define ROI center
                df = self.frame_num - prev.frame_num
                x = prev.x + prev.dxdf * df
                y = prev.y + prev.dydf * df

                # define ROI edges, min/max make sure bounds dont exceed frame
                left = max(x - prev.search_r, 0)
                right = min(x + prev.search_r, frame_W)
                top = min(y + prev.search_r, frame_H)
                bottom = max(y - prev.search_r, 0)

                # if within ROI bounds
                if self.x > left and self.x < right and self.y > bottom and self.y < top:

                    # set remaining parameters
                    self.id = prev.id
                    self.dxdf = (self.x - prev.x)/df
                    self.dydf = (self.y - prev.y)/df
                    self.search_r = 100
                    break
                
                else: # go to next node
                    continue
            
        # if Node not matched to pre-existing id, also catches first ever Node
        if self.id == 0:

            currIDCount = currIDCount + 1
            self.id = currIDCount

            # Guess ROI
            if self.x < sect_1x: # left side
                self.dxdf = 50
            elif self.x > sect_2x: # right side
                self.dxdf = -50
            else:                 # center
                self.dxdf = 0

            if self.y < sect_1y: # low side
                self.dydf = 50
            elif self.y > sect_2y: # high side
                self.dydf = -50
            else:                 # center
                self.dydf = 0
            
            self.search_r = 300 # extra large for first guess 

        return currIDCount
    
    @staticmethod
    def getPrevNodes(allNodes):

        if allNodes == []:
            return []

        prevNodes = []
        index = -1

        # extract most recent node's frame number
        frame_num = allNodes[index].frame_num

        # filter by frame number
        while allNodes[index].frame_num == frame_num:
            # save to output array
            prevNodes.append(allNodes[index])
            index = index - 1

            # if exceeding array index, break loop
            if -index > frame_num:
                break

        return prevNodes
    
    @staticmethod
    def getNodesByID(allNodes, id):
        idNodes = []
        for node in allNodes:
            if node.id == id:
                idNodes.append(node)
        
        return idNodes
        
