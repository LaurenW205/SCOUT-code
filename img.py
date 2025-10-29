import cv2
import numpy as np
from node import Node

def tracer(filename, allNodes, frameDim, idCount):
    # red, orange, yellow, green, blue, purple, pink
    COLORS = [(0,0,255),(0,127,255),(0,255,255),(0,255,0),(255,0,0),(127,0,127),(191,191,255)]

    # extract frame dimensions & initialize black frame
    frame_W = frameDim[0]
    frame_H = frameDim[1]
    traceImg = np.zeros((frame_H, frame_W, 3), dtype=np.uint8)

    # loop through nodes by ID
    for id in range(idCount):

        # extract all nodes of a certain id
        id = id + 1
        nodes = Node.getNodesByID(allNodes, id)

        # set a different color depending on node id
        color = COLORS[(id-1) % 6]

        # loop through nodes to draw points and connecting lines
        for index, node in enumerate(nodes):

            cv2.circle(traceImg, (node.x, node.y), 3, color, -1)

            if index > 0:
                cv2.line(traceImg, (nodes[index-1].x,nodes[index-1].y),(nodes[index].x,nodes[index].y),color, 2)

    # display img
    cv2.imshow("Traced Path", traceImg)
    while cv2.waitKey(1) != ord('q'):
        pass

    # save traced image to file
    cv2.imwrite(filename, traceImg)

    # close window
    cv2.destroyAllWindows()
