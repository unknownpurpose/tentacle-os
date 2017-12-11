import cv2
import numpy as np

def getCornersFromImage(img, board, dictionary):
    mCorners, mIds, _ = cv2.aruco.detectMarkers(img, dictionary)
    if mIds is None: return [], []
    _, boardCorners, ids = cv2.aruco.interpolateCornersCharuco(mCorners, mIds, img, board)
    if boardCorners is None: return [], []
    boardCorners = np.reshape(boardCorners, (len(boardCorners), 2)) # from (N, 1, 2) to (N, 2)
    return ids, boardCorners

def percentVisible(img1, img2, board, dictionary):
    numCorners = len(board.chessboardCorners)
    boardSize = board.getChessboardSize()
    numIds = boardSize[0] * boardSize[1]

    ids1, boardCorners1 = getCornersFromImage(img1, board, dictionary)
    ids2, boardCorners2 = getCornersFromImage(img2, board, dictionary)

    foundIds = np.intersect1d(ids1, ids2)
    percentageFound = float(len(foundIds)) / numIds

    return percentageFound
