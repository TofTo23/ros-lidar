import numpy as np

def calculateVff(pose, target, map_grid, winSize, scale):
    '''start pose, target pose -> vector of direction'''


    # attract vector
    attractVec = np.array([target[0] - pose[0], target[1] - pose[1]], dtype=float)
    distToTarget = np.linalg.norm(attractVec)

    # calculate versor with 0.5 coefficient
    # prevent hitting object
    if distToTarget > 0:
        Fa = attractVec / distToTarget * 0.5
    else:
        Fa = np.array([0.0, 0.0])

    Fr = np.array([0.0, 0.0])

    gridX = int(round(pose[0] * scale))
    gridY = int(round(pose[1] * scale))

    halfWin = int(np.floor(winSize / 2))

    # get map dimensions for bounds checking
    map_rows, map_cols = map_grid.shape

    for ix in range(-halfWin, halfWin + 1):
        for iy in range(-halfWin, halfWin + 1):
            cx = gridX + ix
            cy = gridY + iy

            # bounds checking
            if cx >= 0 and cy >= 0 and cx < map_cols and cy < map_rows:

                # obstacles
                if map_grid[cy, cx] == 1:
                    # euklidean distance in meters
                    dist = np.sqrt(ix**2 + iy**2) / scale
                    
                    # prevent division by zero
                    if dist > 0:
                        repulseMag = 0.05 / (dist**2)
                        Fr[0] = Fr[0] - repulseMag * (ix / dist)
                        Fr[1] = Fr[1] - repulseMag * (iy / dist)

    return Fa, Fr