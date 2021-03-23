import pybullet

def addDebugLine(startPoint, endPoint, color=[0, 0, 0], lineWidth=1, lifeTime=0):
    return pybullet.addUserDebugLine(startPoint, endPoint, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)

def addDebugRectangle(position, quaternion=[0, 0, 0, 1], length=0.2, width=0.1, color=[0, 0, 0], lineWidth=1, lifeTime=0):
    point1, quaternion1 = pybullet.multiplyTransforms(position, quaternion, [+length / 2, +width / 2, 0], [0, 0, 0, 1])
    point2, quaternion2 = pybullet.multiplyTransforms(position, quaternion, [-length / 2, +width / 2, 0], [0, 0, 0, 1])
    point3, quaternion3 = pybullet.multiplyTransforms(position, quaternion, [-length / 2, -width / 2, 0], [0, 0, 0, 1])
    point4, quaternion4 = pybullet.multiplyTransforms(position, quaternion, [+length / 2, -width / 2, 0], [0, 0, 0, 1])
    line1 = addDebugLine(point1, point2, color, lineWidth, lifeTime)
    line2 = addDebugLine(point2, point3, color, lineWidth, lifeTime)
    line3 = addDebugLine(point3, point4, color, lineWidth, lifeTime)
    line4 = addDebugLine(point4, point1, color, lineWidth, lifeTime)
    return [line1, line2, line3, line4]

def addDebugTrajectory(X, Y, Z, color=[0,0,0], lineWidth=1, lifeTime=0):
    trajectoryId = []
    for i in range(len(X)-1):
        pointFrom = [X[i], Y[i], Z[i]]
        pointTo = [X[i+1], Y[i+1], Z[i+1]]
        lineId = pybullet.addUserDebugLine(pointFrom, pointTo, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)
        trajectoryId.append(lineId)
    return trajectoryId

def addDebugTrajectory3D(XYZ, color=[0,0,0], lineWidth=1, lifeTime=0):
    XYZ = XYZ.reshape((3,-1))
    return addDebugTrajectory(XYZ[0],XYZ[1],XYZ[2], color=color, lineWidth=lineWidth, lifeTime=lifeTime)

def addDebugTrajectory2D(XYZ, color=[0,0,0], lineWidth=1, lifeTime=0):
    XYZ = XYZ.reshape((3, -1))
    return addDebugTrajectory(XYZ[0],XYZ[1],XYZ[2]*0.0, color=color, lineWidth=lineWidth, lifeTime=lifeTime)
