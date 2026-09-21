import cv2
import depthai as dai
import numpy as np
import time
import random

import threading
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from colman_motion.arm_control import ArmControl
from colman_motion.scene_manager import SceneManager
from colman_motion.vacuum_control import VacuumControl

STARTING_POSITION = {
    "shoulder_pan_joint": 0 * (np.pi / 180),
    "shoulder_lift_joint": -90 * (np.pi / 180),
    "elbow_joint": 0 * (np.pi / 180),
    "wrist_1_joint": -90 * (np.pi / 180),
    "wrist_2_joint": 0 * (np.pi / 180),
    "wrist_3_joint": 0 * (np.pi / 180),
}

PRE_CAMERA_POSITION = {
    "shoulder_pan_joint": 0.5 * (np.pi / 180),
    "shoulder_lift_joint": -100.5 * (np.pi / 180),
    "elbow_joint": 85.5 * (np.pi / 180),
    "wrist_1_joint": -75.5 * (np.pi / 180),
    "wrist_2_joint": -90.5 * (np.pi / 180),
    "wrist_3_joint": 0.5 * (np.pi / 180),
}

CAMERA_POSITION = {
    "shoulder_pan_joint": 0 * (np.pi / 180),
    "shoulder_lift_joint": -100 * (np.pi / 180),
    "elbow_joint": 85 * (np.pi / 180),
    "wrist_1_joint": -75 * (np.pi / 180),
    "wrist_2_joint": -90 * (np.pi / 180),
    "wrist_3_joint": 0 * (np.pi / 180),
}

# I think a square image looks better here
width = 1000 #640
height = 1000 #480 old
playerColor = "pink"
armColor = "blue"
settingUp = True
playerTurn = False
rawPieces = []
playerPieces = []
armPieces = []
armTurn = False
launchTime = time.time()
lastCheck = time.time()
botCorner = (0,0)
topCorner = (0,0)
row = 0
col = 1
board = [[' ' for _ in range(6)] for _ in range(8)]
playableAreas = [[' ' for _ in range(6)] for _ in range(8)]
APPROACH_OFFSET = 0.1
EE_OFFSET = 0.01
EE_DOWN = (1.0, 0.0, 0.0, 0.0)
class Piece:
    def __init__(self, x, y, row, col, king, team):
        self.x = x
        self.y = y
        self.row = row
        self.col = col
        self.king = king
        self.team = team  #true == player, false == robot arm

    # new default print output for testing sometimes
    def __str__(self):
        # return f"X:{self.x}, Y:{self.y}, R:{self.row}, C:{self.col}"
        if self.team == True:
            return "P"
        else:
            return "B"
        
def printBoard():
    global board

    print("-----------")
    for i in range(len(board)-1, -1, -1):
        for j in range(len(board[i])):
            if board[i][j] == '_':
                print("_", end= "")
                # print("______________________", end= "")
            else:
                print(f"{board[i][j]} ", end= "")
        print()
    print("-----------",flush=True)


def movePiece(initalRow, initialCol, endRow, endCol):

    global arm, vacuum, EE_DOWN, PRE_CAMERA_POSITION, CAMERA_POSITION, veryFast, fast, slow, verySlow

    botRowX = 0.2398202570602908
    topRowX = 0.4568202570602908
    minColY = 0.21078172353256214           # y values are flipped for robot arm move parameters
    maxColY = 0.05078172353256214
    # graveyardMaxY = -0.20078172353256214
    graveyardY = -0.05
    hoverZ = 0.01007029111737157
    grabZ = 0.00207029111737157

    initialX = botRowX + (((topRowX - botRowX) / 7) * initalRow)
    endX = botRowX + (((topRowX - botRowX) / 7) * endRow)
    initalY = minColY - (((minColY - maxColY) / 5) * initialCol)
    endY = minColY - (((minColY - maxColY) / 5) * endCol)

    print(f"initalRow {initalRow},\ninitialCol {initialCol},\nendRow {endRow},\nendCol {endCol}")

    # grab from initial location
    coordinates = [
        initialX, initalY, hoverZ,
    ]
    arm.go_to_pose(*coordinates, *EE_DOWN, fast)

    coordinates[2] = grabZ
    arm.go_to_pose(*coordinates, *EE_DOWN, slow)

    vacuum.grasp()

    coordinates[2] = hoverZ
    arm.go_to_pose(*coordinates, *EE_DOWN, slow)

    # move to new location
    coordinates = [
        endX, endY, hoverZ,
    ]
    arm.go_to_pose(*coordinates, *EE_DOWN, fast)

    coordinates[2] = grabZ
    arm.go_to_pose(*coordinates, *EE_DOWN, slow)

    vacuum.release()

    coordinates[2] = hoverZ
    arm.go_to_pose(*coordinates, *EE_DOWN, slow)

    # if the arm moves more than 1 row and has to get a player piece off the board
    if abs(initalRow - endRow) > 1:
        initialX = (initialX + endX) / 2
        initalY = (initalY + endY) / 2

        coordinates = [
            initialX, initalY, hoverZ,
        ]
        arm.go_to_pose(*coordinates, *EE_DOWN, fast)

        coordinates[2] = grabZ
        arm.go_to_pose(*coordinates, *EE_DOWN, slow)

        vacuum.grasp()

        coordinates[2] = hoverZ
        arm.go_to_pose(*coordinates, *EE_DOWN, slow)

        coordinates[1] = graveyardY
        coordinates[2] = hoverZ + 0.03
        arm.go_to_pose(*coordinates, *EE_DOWN, veryFast)

        vacuum.release()


    arm.go_to_joint_pose(PRE_CAMERA_POSITION, veryFast)
    arm.go_to_joint_pose(CAMERA_POSITION, verySlow)


def armPlay():

    global board, playerPieces, playableAreas, armPieces, botCorner, topCorner, playerTurn, armTurn

    # save in this format to make things easier later
    # index of armPiece, initial row, initial col, result row, result col, boolean for if the move removes a player piece
    legalArmMoves = []

    for i in range(len(armPieces)):
        # if you can look left
        if armPieces[i].col > 0 and armPieces[i].row < 7:
            if board[armPieces[i].row + 1][armPieces[i].col - 1] == '_':
                legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row + 1), (armPieces[i].col - 1), False))

        # new work 
        # if you can look right
        if armPieces[i].col < 5 and armPieces[i].row < 7:
            if board[armPieces[i].row + 1][armPieces[i].col + 1] == '_':
                legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row + 1), (armPieces[i].col + 1), False))


        # basic left point
        if armPieces[i].col > 1 and armPieces[i].row < 6:
            if board[armPieces[i].row + 2][armPieces[i].col - 2] == '_' and board[armPieces[i].row + 1][armPieces[i].col - 1] != '_':
                type(Piece)
                if board[armPieces[i].row + 1][armPieces[i].col - 1].team == True:
                    legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row + 2), (armPieces[i].col - 2), True))


        # basic right point
        if armPieces[i].col < 4 and armPieces[i].row < 6:
            if board[armPieces[i].row + 2][armPieces[i].col + 2] == '_' and board[armPieces[i].row + 1][armPieces[i].col + 1] != '_':
                if board[armPieces[i].row + 1][armPieces[i].col + 1].team == True:
                    legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row + 2), (armPieces[i].col + 2), True))


        # king exclusive moves
        if armPieces[i].king == True:

            #king left
            if armPieces[i].col > 0 and armPieces[i].row > 0:
                if board[armPieces[i].row - 1][armPieces[i].col - 1] == '_':
                    legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row - 1), (armPieces[i].col - 1), False))

            # king right
            if armPieces[i].col < 5 and armPieces[i].row > 0:
                if board[armPieces[i].row - 1][armPieces[i].col + 1] == '_':
                    legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row - 1), (armPieces[i].col + 1), False))


            # king left point
            if armPieces[i].col > 1 and armPieces[i].row > 1:
                if board[armPieces[i].row - 2][armPieces[i].col - 2] == '_' and board[armPieces[i].row - 1][armPieces[i].col - 1] != '_':
                    if board[armPieces[i].row - 1][armPieces[i].col - 1].team == True:
                        legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row - 2), (armPieces[i].col - 2), True))


            # king right point
            if armPieces[i].col < 4 and armPieces[i].row > 1:
                if board[armPieces[i].row - 2][armPieces[i].col + 2] == '_' and board[armPieces[i].row - 1][armPieces[i].col + 1] != '_':
                    if board[armPieces[i].row - 1][armPieces[i].col + 1].team == True:
                        legalArmMoves.append((i, armPieces[i].row, armPieces[i].col, (armPieces[i].row - 2), (armPieces[i].col + 2), True))


    # add physical moves and delete logic here
    if len(legalArmMoves) > 0:

        chosenMove = legalArmMoves[random.randrange(len(legalArmMoves))]

        movePiece(chosenMove[1], chosenMove[2], chosenMove[3], chosenMove[4])

        armPieces[chosenMove[0]].row = chosenMove[3]
        armPieces[chosenMove[0]].col = chosenMove[4]
        armPieces[chosenMove[0]].x = playableAreas[chosenMove[3]][chosenMove[4]][0]
        armPieces[chosenMove[0]].y = playableAreas[chosenMove[3]][chosenMove[4]][1]
        board[chosenMove[1]][chosenMove[2]] = '_'
        board[chosenMove[3]][chosenMove[4]] = armPieces[chosenMove[0]]


        if chosenMove[5] == True:
            deleteRow = int((chosenMove[1] + chosenMove[3]) / 2)
            deleteCol = int((chosenMove[2] + chosenMove[4]) / 2)

            for i in range(len(playerPieces)):
                if playerPieces[i].row == deleteRow and playerPieces[i].col == deleteCol:
                    del playerPieces[i]
                    break
            board[deleteRow][deleteCol] = '_'

        print(legalArmMoves,flush=True)
        print(f"Chosen move: {chosenMove}",flush=True)
        printBoard()


    armTurn = False
    playerTurn = True

def checkPlayerMovement(contours):

    global board, playerPieces, armPieces, botCorner, topCorner, playerTurn, armTurn

    amountFound = 0
    piecesFound = [False] * len(playerPieces)
    foundIndex = -1
    potentialX = 0
    potentialY = 0
    legalMove = False
    for cnt in contours:
        found = False
        area = cv2.contourArea(cnt)
        if area > 400:  # ignore tiny noise
            x, y, w, h = cv2.boundingRect(cnt)
            cX, cY = x + int(w / 2), y + int(h / 2)
            # check if a contour is within play
            if cX < botCorner[0] and cX > topCorner[0] and cY < botCorner[1] and cY > topCorner[1]:

                #check if the contour matches the location of a piece
                for i in range(len(playerPieces)):
                    if abs(cX - playerPieces[i].x) < 30 and abs(cY - playerPieces[i].y) < 30:
                        piecesFound[i] = True
                        amountFound += 1
                        found = True
                        break
                if found == False:
                    potentialX = cX
                    potentialY = cY


    #if all pieces were found within play, but exactly one was not in any of the currently expected positions
    if potentialY != 0 and amountFound == len(playerPieces) - 1:
        for i in range(len(playerPieces)):
            if piecesFound[i] == False:
                # get the index of the singular piece
                foundIndex = i

    if foundIndex > -1:
        for i in range(len(board)):
            for j in range(len(board[i])):
                # if the spot on the board is blank and playable
                if board[i][j] == '_': 
                    # if the scanned contour is close enough to the center of a playable area
                    if abs(potentialX - playableAreas[i][j][0]) < 30 and abs(potentialY - playableAreas[i][j][1]) < 30:

                        # comparing the blank spot that contour is currently positioned in with the previous position of the one missing checkers piece

                        # basic downward movement 1 unit
                        if playerPieces[foundIndex].row == i + 1 and (playerPieces[foundIndex].col == j - 1 or playerPieces[foundIndex].col == j + 1):
                            legalMove = True

                            # if piece earned the title of king
                            if i == 0:
                                playerPieces[foundIndex].king = True

                        # player killed arm piece downward left
                        if legalMove == False and playerPieces[foundIndex].row == i + 2 and playerPieces[foundIndex].col == j - 2 and board[i+1][j-1].team == False:
                            legalMove = True
                            for k in range(len(armPieces)):
                                if armPieces[k].row == i+1 and armPieces[k].col == j-1:
                                    del armPieces[k]
                                    break
                            board[i+1][j-1] = '_'

                        # player killed arm piece downward right
                        if legalMove == False and playerPieces[foundIndex].row == i + 2 and playerPieces[foundIndex].col == j + 2 and board[i+1][j+1].team == False:
                            legalMove = True
                            for k in range(len(armPieces)):
                                if armPieces[k].row == i+1 and armPieces[k].col == j+1:
                                    del armPieces[k]
                                    break
                            board[i+1][j+1] = '_'

                        # double jumps downwards
                        # come back to this and keep indexes from going out of bounds during if statements


                        # royal moves
                        if playerPieces[foundIndex].king == True:

                            # basic upward movement 1 unit
                            if playerPieces[foundIndex].row == i - 1 and (playerPieces[foundIndex].col == j - 1 or playerPieces[foundIndex].col == j + 1):
                                legalMove = True

                            # add rest later


                    if legalMove == True:
                        armTurn = True
                        playerTurn = False
                        board[playerPieces[foundIndex].row][playerPieces[foundIndex].col] = '_'
                        playerPieces[foundIndex].row = i
                        playerPieces[foundIndex].col = j
                        playerPieces[foundIndex].x = playableAreas[i][j][0]
                        playerPieces[foundIndex].y = playableAreas[i][j][1]
                        board[i][j] = playerPieces[foundIndex]
                        print(f"player moved to {i},{j}",flush=True)
                        printBoard()
                        break
                        # add a second check for kings moving upwards
            if legalMove == True:
                break



# using the inital positions of the checkers pieces to define most of the playable areas inside createBoard()
# this is just for the areas in the middle that aren't initially filled with pieces
def definePlayableAreas():

    global board, playableAreas

    # figure out the y value of the 2 missing rows
    averageRow2y = (board[2][1].y + board[2][3].y + board[2][5].y)/3
    averageRow5y = (board[5][0].y + board[5][2].y + board[5][4].y)/3
    rowDifference = (averageRow2y - averageRow5y) / 3
    row3y = averageRow2y - rowDifference
    row4y = averageRow5y + rowDifference

    for i in range (0, 6, 2):
        playableAreas[3][i] = ((board[1][i].x + board[5][i].x + board[7][i].x) / 3, row3y)
    for i in range (1, 7, 2):
        playableAreas[4][i] = ((board[0][i].x + board[2][i].x + board[6][i].x) / 3, row4y)


def createBoard(cX, cY):
    
    global settingUp, playerPieces, armPieces, botCorner, topCorner, row, col, board, playerTurn, playableAreas

    duplicate = False
    for i in range(len(playerPieces)):
        if abs(cX - playerPieces[i].x) < 30 and abs(cY - playerPieces[i].y) < 30:
            duplicate = True
            break
        
    for i in range(len(armPieces)):
        if abs(cX - armPieces[i].x) < 30 and abs(cY - armPieces[i].y) < 30:
            duplicate = True
            break
        
    if duplicate == False: # if unique
        if cY < height // 2: # if on player's side of board
            rawPieces.append(Piece(cX, cY, 0, 0, False, True))                            
        else:
            rawPieces.append(Piece(cX, cY, 0, 0, False, False))

        # sorting pieces 3 at a time because the camera reads bottom up right to left and sometimes one piece is a pixel too low and gets read earlier than expected
        if len(rawPieces) == 3:
            # sorting the row by x value
            rawPieces.sort(key=lambda rawPieces: rawPieces.x)
            for piece in rawPieces:
                piece.row = row
                piece.col = col
                col = col + 2

            row = row + 1
            if row == 3:
                row = row + 2

            if col == 7:
                col = 0
            else:
                col = 1

            if len(armPieces) > 8: #if piece belongs to player. arm pieces get scanned first so this works
                for i in range (3):
                    playerPieces.append(rawPieces[i])
            else:
                for i in range (3):
                    armPieces.append(rawPieces[i])

            rawPieces.clear()


    # organize pieces onto the board
    if len(armPieces) + len(playerPieces) == 18:
        botCorner = (armPieces[2].x + 45, armPieces[2].y + 45)
        topCorner = (playerPieces[6].x - 45, playerPieces[6].y - 45)
        # print all pieces to make sure they're correct
        for i in range(len(armPieces)):
            print(f"{i}: {armPieces[i]}")
        for i in range(len(playerPieces)):
            print(f"{i}: {playerPieces[i]}",flush=True)

        for piece in playerPieces + armPieces:
            board[piece.row][piece.col] = piece
            playableAreas[piece.row][piece.col] = (piece.x + 0,piece.y + 0) #0s may or may not be needed 
        for i in range (0, 6, 2):
            board[3][i] = '_'
        for i in range (1, 7, 2):
            board[4][i] = '_'

        printBoard()
        definePlayableAreas()

        settingUp = False
        playerTurn = True




def track_color(mask, label, bgr_color):
    global settingUp, launchTime, botCorner, topCorner, lastCheck, playerColor

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # occassionally check if the player made a move and then do the proper updates
    if time.time() - lastCheck > 0.8 and playerTurn == True and label == playerColor:
        checkPlayerMovement(contours)
        lastCheck = time.time()


    if time.time() - lastCheck > 0.8 and armTurn == True:
        print("Arm turn lolgic here",flush=True)
        armPlay()
        lastCheck = time.time()


    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 400:  # ignore tiny noise
            x, y, w, h = cv2.boundingRect(cnt)
            cX, cY = x + int(w / 2), y + int(h / 2)

            if settingUp and time.time() - launchTime >= 6.5: 
                # initial x and y values are wildly off if you don't wait a few seconds
                createBoard(cX, cY)

                
            cv2.circle(frame, (cX, cY), 4, bgr_color, -1)



# define what is pink and what is blue 
pink_lower = np.array([165, 110, 180])
pink_upper = np.array([180, 200, 255])

blue_lower = np.array([90, 80, 80])
blue_upper = np.array([130, 255, 255])

# variables to control the arm and vacuum
rclpy.init()
ur = MoveItPy(node_name="moveit_py")

arm = ArmControl(ur)
scene = SceneManager(ur)
vacuum = VacuumControl()

executor = MultiThreadedExecutor()
executor.add_node(arm)
executor.add_node(scene)
executor.add_node(vacuum)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

# Setup collision scene and planners
scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))
veryFast = PlanRequestParameters(ur, "ompl")
veryFast.max_velocity_scaling_factor = 0.99
veryFast.max_acceleration_scaling_factor = 0.99
fast = PlanRequestParameters(ur, "ompl")
fast.max_velocity_scaling_factor = 0.8
fast.max_acceleration_scaling_factor = 0.8
slow = PlanRequestParameters(ur, "ompl")
slow.max_velocity_scaling_factor = 0.2
slow.max_acceleration_scaling_factor = 0.2
verySlow = PlanRequestParameters(ur, "ompl")
verySlow.max_velocity_scaling_factor = 0.01
verySlow.max_acceleration_scaling_factor = 0.01
vacuum.release()

# get in the position
# arm.go_to_joint_pose(STARTING_POSITION, veryFast) # if I ever wanna reset the arm position from the computer instead
arm.go_to_joint_pose(PRE_CAMERA_POSITION, veryFast)
arm.go_to_joint_pose(CAMERA_POSITION, verySlow)
launchTime = time.time()

# make a pipeline
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()

    videoQueue = cam.requestOutput((width, height)).createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        videoIn = videoQueue.get()
        if videoIn is None:
            continue

        frame = videoIn.getCvFrame()

        # convert BGR frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # create binary masks for pink and blue
        mask_pink = cv2.inRange(hsv, pink_lower, pink_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

        # clean up minor noise using morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)


        # track both pink and blue pieces
        # this assumes blue pieces are on the arm side and pink pieces are on the player side. swap the order of these two lines if you swap color choices
        track_color(mask_blue, armColor, (255, 0, 0))
        track_color(mask_pink, playerColor, (147, 20, 255))


        # visual aids for where to position the checkerboard during startup before the game starts
        if settingUp:
            # nothing will work if pieces are not on opposite sides of this horizontal line during startup
            # I use the position of the line to sort pieces into teams
            cv2.line(frame, (0, height // 2), (width, height // 2), (0, 0, 255), 1)
            # this line kinda helps center things but is not as important
            cv2.line(frame, (width // 2, 0), (width // 2, width), (0, 0, 255), 1) 

            # these dots kinda line up with the corners but are not as useful as the main lines imo
            cv2.circle(frame, (194, 94), radius = 5, color = (0, 255, 0), thickness = -1)
            cv2.circle(frame, (702, 90), radius = 5, color = (0, 255, 0), thickness = -1)
            cv2.circle(frame, (803, 193), radius = 5, color = (0, 255, 0), thickness = -1)
            cv2.circle(frame, (199, 805), radius = 5, color = (0, 255, 0), thickness = -1)
            cv2.circle(frame, (300, 906), radius = 5, color = (0, 255, 0), thickness = -1)
            cv2.circle(frame, (805, 905), radius = 5, color = (0, 255, 0), thickness = -1)

        # visual aid for playing area
        else:
            cv2.rectangle(frame, botCorner, topCorner, (0, 255, 0), 2)

        cv2.imshow("Checkers", frame)

        # exit conditions
        if cv2.waitKey(1) == ord("q") or (settingUp == False and (len(armPieces) == 0 or len(playerPieces) == 0)):
            break

cv2.destroyAllWindows()
vacuum.release()
scene.clear_scene()
executor.shutdown()
rclpy.shutdown()