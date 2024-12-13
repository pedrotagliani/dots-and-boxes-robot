import cv2
import dnbpy
from modules import utils
import pickle
import numpy as np
import keyboard
from modules import open_loop_traj_control
from time import sleep

class DnbGame():
    # boardDotSize (tuple) --> (dotsWidth, dotsHeight)
    # playerName (string) --> 'Pedro', for example
    # difficulty (string) --> 'easy', 'medium' or 'hard'
    # firstPlayer (int) --> 0 (player, default value) or 1 (robot)
    # distanceBetweenDots (int) --> Distance in centimetre. For example: 3.6
    # markerSizeInCM (int) --> Size of the marker in centimetre. For example: 3.8
    # camera (object) --> Instance of the custom myCamera class
    def __init__(self, boardDotSize, playerName, difficulty,
                 distanceBetweenDots, markerSizeInCM, 
                 camera, firstPlayer = 0):
        
        # Logical configuration
        self.playerName = playerName
        self.difficulty = difficulty
        self.firstPlayer = firstPlayer
        self.players = [self.playerName, 'Robot']
        self.boardDetected = False # If the board was detected --> self.boardDetected = True

        # Initialize the camera parameters from an instance of the MyCamera class
        self.camera = camera # Camera object of the MyCamera class
        self.frameWidth = camera.frameWidth
        self.frameHeight = camera.frameHeight
        self.cameraMatrix = camera.cameraMatrix
        self.distCoeffs = camera.distCoeffs

        # Board's configuration
        self.boardDotSize = boardDotSize
        self.dotsWidth = self.boardDotSize[0]
        self.dotsHeight = self.boardDotSize[1]
        self.boardNumColumns = self.dotsWidth - 1
        self.boardNumRows = self.dotsHeight - 1
        self.boardSize = (self.boardNumRows, self.boardNumColumns) # A board in the dnbpy library is described using the convention n x m, where n is the number of rows, and m is the number of columns
        self.boardTotalLines = self.dotsWidth*self.boardNumRows + self.dotsHeight*self.boardNumColumns # If 6x6 --> 60 lines
        self.distanceBetweenDots = distanceBetweenDots
        self.markerSizeInCM = markerSizeInCM # Size of the marker in centimetre

        # Create a new game instances
        self.gameDnbpyLib = dnbpy.Game(self.boardSize, self.players)

        # Change the current player (default value is 0)
        self.gameDnbpyLib._current_player = self.firstPlayer

        # Select the AI according to the difficulty
        if self.difficulty == 'easy':
            self.aiAgent = dnbpy.RandomPolicy()
        elif self.difficulty == 'medium':
            self.aiAgent = dnbpy.Level2HeuristicPolicy(self.boardSize)
        elif self.difficulty == 'hard':
            self.aiAgent = dnbpy.Level3MinimaxPolicy(self.boardSize, depth=2, update_alpha=True)
        
        # Define the ArUco detector
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.arucoParams =  cv2.aruco.DetectorParameters()
        # self.arucoParams.minMarkerDistanceRate = 0.03
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Matrices for storing the dnbpy convention
        self.horizontalLinesMatrix, self.verticalLinesMatrix = utils.get_dnb_matrix_convention(self.boardNumRows, self.boardNumColumns, self.boardTotalLines)

        # Matrices for storing the pair of points that are involved in the creation of a particular line
        self.pointsHorizontalLinesMatrix, self.pointsVerticalLinesMatrix = utils.dnb_conv_to_matrix_points(self.boardNumRows, self.boardNumColumns, self.dotsHeight, self.dotsWidth)
        
        # Create a matrix to store all distances from the base frame of the robotic arm to the points on the whiteboard
        # First, take into account the distance from the robot base frame to the first point on the whiteboard (consider the orientation of the base frame's axis)
        # If this is not clear, refer to the image 'v2_Robot to circles.png'

        # xbp1 = 23.4 # CHECK
        # ybp1 = -5.4 # CHECK
        # zbp = 4 # CHECK

        # self.tbpointsMatrix = utils.get_distance_matrix_from_robot_to_points(xbp1, ybp1, zbp, self.dotsHeight, self.dotsWidth, self.distanceBetweenDots)
        
        z = 6.4
        pitchAngle = 8.0

        self.tbpointsMatrix = np.array(
            # First row
            [[[23.8 - 0.3, -5.4 + 0.4,  z + 0.6, pitchAngle],
            [23.4 + 0.4, -1.8 + 0.4,  z + 0.6, pitchAngle],
            [23.4 + 0.6,  1.8 + 0.8,  z + 0.6, pitchAngle],
            [23.4 + 0.2,  5.4 + 0.7, z + 0.6, pitchAngle]],

            # Second row
            [[27.0 - 0.5, -5.4 + 0.4,  z + 0.3, pitchAngle],
            [27.0 - 0.3, -1.8 + 0.5,  z + 0.2, pitchAngle],
            [27.0 - 0.1,  1.8 + 0.5,  z + 0.3, pitchAngle],
            [27.0 - 0.6,  5.4 + 0.6,  z + 0.3, pitchAngle]],

            # Third row
            [[30.6 - 1.5, -5.4 + 0.8,  z - 0.3, pitchAngle],
            [30.6 - 1.4, -1.8 + 1,  z - 0.3, pitchAngle],
            [30.6 - 1.2,  1.8 + 0.9,  z - 0.3, pitchAngle],
            [30.6 - 1.5,  5.4 + 0.6,  z - 0.3, pitchAngle]]]
        )

        # The width of the wood is 21.4cm
        # The length of the board (excluding the black frames) is 33cm.
        # To center it, there is 5.8cm on each side.

        # Save the previous frame to compare to the possible current frame
        self.previousFrame = None
        self.AcceptedDiffFrameError = 0.70
        # With a threshold of 0.70, we can accept small disturbances. However, any disturbances must be outside the gameboard

        self.averageTcpMatrixTransformed = None
        self.transformationMatrix = None

    def has_finished(self):
        return self.gameDnbpyLib.is_finished() # return True or False
    
    # This function is used to detect the game board. It has to be used once, but the camera needs to remain fixed in the same position the whole match
    def detect_board(self):

        # Variables to store the tcp matrices when the whiteboard is being detected
        setOfTcpointsMatrix = []
        countSetOfTcpMatrix = 0
        maxSetOfTcpMatrix = 30

        # An array that stores the values obtained from performing element-wise operations on matrices
        averageTcpMatrix = np.zeros((self.dotsHeight,self.dotsWidth,2), dtype=np.float32)

        # Update board detection status (average)
        self.boardDetected = False

        while self.boardDetected == False:

            # Obtain frame
            frame = self.camera.get_frame()

            if frame is not None:

                # Make copies of the original frame
                frameCopy = frame.copy()
                frameCopy2 = frame.copy()
                frameCopy3 = frame.copy()
                frameCopy4 = frame.copy()

                # Convert to gray scale
                gray = cv2.cvtColor(frameCopy, cv2.COLOR_BGR2GRAY)

                # Detect ArUco markers in the video frame
                corners, ids, rejected = self.arucoDetector.detectMarkers(gray)

                # Draw ArUco markers detected
                cv2.aruco.drawDetectedMarkers(frameCopy, corners)

                # If at least one ArUco markers is detected, then execute the following code; otherwise (None cases) don't execute
                if ids is not None:

                    # print(len(corners)) # n
                    # print(corners[0].shape) # (n,1,4,2) -> n is in the tuple layer (in this case, n = 0 --> corners[0].shape)
                    # print(ids.shape) # (n,1)

                    # Flatten the ArUco IDs and make it a list
                    ids = ids.flatten().tolist()
                    # print(ids.shape) # (n,)

                    # Display how many ArUco markers are being detected
                    markersFoundString = f'{len(ids)} marcadores encontrados.'
                    cv2.putText(frameCopy,markersFoundString,(20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)

                    # If the IDs 0, 1, 2 and 3 are being detected and no other IDs are interfering, then execute the if statement
                    if (0 in ids) and (1 in ids) and (2 in ids) and (3 in ids) and len(ids) == 4: # This is done to avoid false detections (for example, [2,1,3,38])
                        
                        # Array to store the left corner from all the markers
                        markersLeftCorners = []

                        # Here, I'll store the pose of each marker with respect to the camera frame
                        markersPose = []

                        # Iterate over every detected ArUco marker
                        for markerCorner, markerId in zip(corners, ids):

                            # Subpixel corner detection
                            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
                            markerCornerUnmodified = cv2.cornerSubPix(cv2.cvtColor(frameCopy, cv2.COLOR_BGR2GRAY), markerCorner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

                            # Reshape the array
                            markerCorner2 = markerCornerUnmodified.reshape((4, 2))

                            # Classify each corner of the marker
                            (topLeft, topRight, bottomRight, bottomLeft) = markerCorner2

                            # Append the top left corner of the ArUco marker in the previously created array
                            markersLeftCorners.append([topLeft[0],topLeft[1]])

                            # Draw the top left corner and display the id number of the marker
                            cv2.circle(frameCopy,(int(topLeft[0]),int(topLeft[1])),5,(0,0,255),thickness=-1)
                            cv2.putText(frameCopy, str(markerId), (int(bottomRight[0]+10), int(bottomRight[1]+10)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                            # Find the pose of each marker respect to the camera
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCornerUnmodified, self.markerSizeInCM, self.cameraMatrix, self.distCoeffs)

                            # Flatten the position from the camera to the marker
                            tvec = tvec.flatten()

                            # Draw frame axis:
                            cv2.drawFrameAxes(frameCopy, self.cameraMatrix, self.distCoeffs, rvec, tvec, length=8, thickness=4)

                            # I am interested in saving the pose of each marker. Later, I'll relate each one of them with every circle in the real matrix
                            markersPose.append([rvec,tvec])

                        # Overlay to draw the polygon
                        overlay = frame.copy()

                        # List to store the top left marker corner sorted by ID number
                        markersLeftCornersSorted = [None, None, None, None]

                        # List to store the markers' pose sorted by ID number
                        markersPoseSorted = [None, None, None, None]

                        for i in range(4): #0,1,2,3
                            index = ids.index(i) # if ids = [3,1,2] --> i = 3 --> index = 0
                            markersLeftCornersSorted[i] = markersLeftCorners[index]
                            markersPoseSorted[i] = markersPose[index]

                        # Convert from list to array so I can use cv2.fillPolly
                        markersLeftCornersSorted = np.array(markersLeftCornersSorted)

                        # https://stackoverflow.com/questions/17241830/opencv-polylines-function-in-python-throws-exception
                        cv2.fillPoly(overlay, pts = np.int32([markersLeftCornersSorted]), color=(255,215,0))

                        # Blend the overlay and the original frame with cv2.addWeighted
                        alpha = 0.4  # Transparency factor
                        beta = 1 - alpha # Transparency factor
                        gamma = 0 # Transparency factor
                        # frameWithOverlay = cv2.addWeighted(frame.copy(), alpha, overlay, beta, gamma)

                        # Destination coordinates sorted by top left, top right, bottom left, bottom right:
                        ptsDestination = np.float32([[0,0],[self.frameWidth,0],[0,self.frameHeight],[self.frameWidth,self.frameHeight]])

                        # The source points will be static. In other words, they'll ve related with the same ArUco Markers
                        # It doesn't matter where the camera is placed as long as the markers are correctly detected
                        # It's important to keep in mind that the top left corner of the markers are already sorted by IDs in the array called markersLeftCornersSorted

                        # Source coordinates sorted by top left, top right, bottom left, bottom right:
                        ptsSource = np.float32([markersLeftCornersSorted[0], markersLeftCornersSorted[3], markersLeftCornersSorted[1], markersLeftCornersSorted[2]])
                        # If you take a look at the image with the marker IDs displayed, it's easy to to find the correct order in ptsSource

                        # Obtain the transfomation matrix:
                        transformationMatrix = cv2.getPerspectiveTransform(ptsSource,ptsDestination)

                        # Apply the transformation matrix in the original frame
                        # boardFrame = cv2.warpPerspective(frame.copy(),transformationMatrix,(self.frameWidth,self.frameHeight))

                        # Now, I should find where the dots are placed on the whiteboard
                        # The markers are fixed (they are always in the same place), so the distance from every marker to the circles on the whiteboard will always be the same
                        # I'm gonna reference every point to each marker
                        # If I take the distance from one marker to only one circle, I can calculate the rest because they are all separated by approximately 3.6cm

                        # Distance from marker with ID 0 to the first circle ---> (-0.8cm, 6.2cm, 0cm)
                        x0p = -0.8 + 3.6
                        y0p = 6.2 + 3.6
                        z0p = 0

                        # I know the pose of this marker with respect to the camera. Hence I can construct the homogeneous transformation matrix that maps points from the marker’s coordinate frame to the camera’s coordinate frame

                        # The rotation obtained is expressed in Rodrigues’ angles; therefore, I will convert it to a rotation matrix (camera to 0)
                        Rc0 = cv2.Rodrigues(markersPoseSorted[0][0])[0] # It returns a tuple with a 3x3 matrix inside corresponding to the rotation matrix

                        # The translation vector from the camera to the marker 0 was already obtained (camera to 0)
                        tc0 = np.array(markersPoseSorted[0][1]).flatten()

                        # Construct the HTM:
                        # Ac0 = np.array([[Rc0[0][0], Rc0[0][1], Rc0[0,2], tc0[0]],
                        # 				[Rc0[1][0], Rc0[1][1], Rc0[1,2], tc0[1]],
                        # 				[Rc0[2][0], Rc0[2][1], Rc0[2,2], tc0[2]],
                        # 				[0, 0, 0, 1]])

                        # Translation vector from the marker 0 to the nearest circle (0 to point)
                        t0p = np.array([x0p, y0p, z0p])
                        # t0pHomogeneous = np.array([x0p, y0p, z0p, 1])

                        # Translation vector from the camera to the point in the real world coordinate
                        # tcp = np.matmul(Ac0,t0pHomogeneous)[:-1]

                        # Project the 3D point obtained to an image plane
                        # I've already transformed the points and they're camera-local. Therefore, I should pass empty/all-zero rvec and tvec
                        # tcpProjected2, _ = cv2.projectPoints(tcp, (0,0,0),(0,0,0), cameraMatrix, distCoeffs)

                        # https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c
                        # https://stackoverflow.com/questions/73340550/how-does-opencv-projectpoints-perform-transformations-before-projecting

                        # It's not necessary to do the point transformation manually
                        # If I pass the translation vector from marker 0 to point and the pose of the marker 0 respect to the camera, then the function returns the projection
                        # of the 3D translation vector (from the camera to the circle) to the image plane (2D projection)
                        tcpProjected0, _ = cv2.projectPoints(t0p, markersPoseSorted[0][0], markersPoseSorted[0][1], self.cameraMatrix, self.distCoeffs)

                        # Reshape the translation vector to ease its manipulation
                        tcpProjected0 = np.array(tcpProjected0).flatten()
                        # print(tcpProjected)

                        # Draw the point in the frame
                        cv2.circle(frameCopy,(int(tcpProjected0[0]),int(tcpProjected0[1])),5,(0,255,0),thickness=-1)

                        # Distance from marker with ID 1 to the first circle
                        x1p = -3.5 - 3.6*5 + 3.6 + 0.2
                        y1p = 6.6 + 0.2 + 3.6 - 0.4
                        z1p = 0

                        # Translation vector from the marker 1 to the nearest circle (1 to point)
                        t1p = np.array([x1p, y1p, z1p])

                        # If I pass the translation vector from marker 1 to point and the pose of the marker 1 respect to the camera, then the function returns the projection
                        # of the 3D translation vector (from the camera to the circle) to the image plane (2D projection)
                        tcpProjected1, _ = cv2.projectPoints(t1p, markersPoseSorted[1][0], markersPoseSorted[1][1], self.cameraMatrix, self.distCoeffs)

                        # Reshape the translation vector to ease its manipulation
                        tcpProjected1 = np.array(tcpProjected1).flatten()

                        # Draw the point in the frame
                        cv2.circle(frameCopy,(int(tcpProjected1[0]),int(tcpProjected1[1])),5,(0,255,0),thickness=-1)

                        # Distance from marker with ID 2 to the first circle
                        x2p = -6.1 - 3.6*5 + 0.2 + 3.6 + 0.2
                        y2p = 3.5 + 3.6*5 + 0.2 - 3.6 - 0.2
                        z2p = 0

                        # Translation vector from the marker 2 to the nearest circle (2 to point)
                        t2p = np.array([x2p, y2p, z2p])

                        # If I pass the translation vector from marker 2 to point and the pose of the marker 2 respect to the camera, then the function returns the projection
                        # of the 3D translation vector (from the camera to the circle) to the image plane (2D projection)
                        tcpProjected2, _ = cv2.projectPoints(t2p, markersPoseSorted[2][0], markersPoseSorted[2][1], self.cameraMatrix, self.distCoeffs)

                        # Reshape the translation vector to ease its manipulation
                        tcpProjected2 = np.array(tcpProjected2).flatten()

                        # Draw the point in the frame
                        cv2.circle(frameCopy,(int(tcpProjected2[0]),int(tcpProjected2[1])),5,(0,255,0),thickness=-1)

                        # Distance from marker with ID 3 to the first circle
                        x3p = -6.4 - 3.6*5 - 0.4 + 3.6 + 0.2
                        y3p = 0.4 - 3.6
                        z3p = 0

                        # Translation vector from the marker 3 to the nearest circle (3 to point)
                        t3p = np.array([x3p, y3p, z3p])

                        # If I pass the translation vector from marker 3 to point and the pose of the marker 3 respect to the camera, then the function returns the projection
                        # of the 3D translation vector (from the camera to the circle) to the image plane (2D projection)
                        tcpProjected3, _ = cv2.projectPoints(t3p, markersPoseSorted[3][0], markersPoseSorted[3][1], self.cameraMatrix, self.distCoeffs)

                        # Reshape the translation vector to ease its manipulation
                        tcpProjected3 = np.array(tcpProjected3).flatten()

                        # Draw the point in the frame
                        cv2.circle(frameCopy,(int(tcpProjected3[0]),int(tcpProjected3[1])),5,(0,255,0),thickness=-1)

                        # Apply the transformation matrix in the frame to draw the four points
                        # testMarkerstoPointsFrame = cv2.warpPerspective(frameCopy,transformationMatrix,(self.frameWidth, self.frameHeight))

                        # The purpose now is to find the location every 3D points with respect to the camera frame and then project them to the image plane (2D)
                        # Each point is separated by 3.6cm, so I'll use tnp (n = 0,1,2,3) as an initial reference

                        # Matrix to store all the translation vectors from the camera to the selected point
                        tcpointsMatrix = np.zeros((self.dotsHeight,self.dotsWidth,4,2), dtype=np.float32)
                        # The 4 in np.zeros correspond to each one of the ArUco markers
                        # The 2 in np.zeros is to store the points coordinates (x,y)

                        # Iterate over all markers
                        for markerId in range(4):
                            # Auxiliar variables
                            xAux = 0
                            yAux = 0

                            if markerId == 0:
                                # Let's iterate over the 6x6 matrix
                                # The location of the translation vectors from the marker 0 to each circle on the whiteboard is calculated using the first t0p as reference
                                for row in range(self.dotsHeight):

                                    for column in range(self.dotsWidth):

                                        # According to the marker 0’s reference frame, the points that share a row will have the same value in x and different values in y (considering the 3.6cm separation between them)
                                        # If you don't get it, check the image where the coordinates of marker 0 are drawn

                                        currentPoint = np.array([x0p + xAux, y0p + yAux, z0p])

                                        currentTcpProjected, _ = cv2.projectPoints(currentPoint, markersPoseSorted[markerId][0], markersPoseSorted[markerId][1], self.cameraMatrix, self.distCoeffs)
                                        currentTcpProjected = np.array(currentTcpProjected).flatten()
                                        tcpointsMatrix[row][column][markerId] = currentTcpProjected

                                        # Draw the point in a clean frame
                                        cv2.circle(frameCopy2,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)

                                        # Actualize the column value
                                        yAux += self.distanceBetweenDots

                                    # Actualize the row value
                                    xAux += self.distanceBetweenDots

                                    yAux = 0
                                xAux = 0

                            elif markerId == 1:
                                # Let's iterate over the 6x6 matrix
                                # The location of the translation vectors from the marker 1 to each circle on the whiteboard is calculated using the first t1p as reference
                                for row in range(self.dotsHeight):

                                    for column in range(self.dotsWidth):

                                        # According to the marker 1’s reference frame, the points that share a row will have the same value in x and different values in y (considering the 3.6cm separation between them)
                                        # If you don't get it, check the image where the coordinates of marker 1 are drawn

                                        currentPoint = np.array([x1p + xAux, y1p + yAux, z1p])

                                        currentTcpProjected, _ = cv2.projectPoints(currentPoint, markersPoseSorted[markerId][0], markersPoseSorted[markerId][1], self.cameraMatrix, self.distCoeffs)
                                        currentTcpProjected = np.array(currentTcpProjected).flatten()
                                        tcpointsMatrix[row][column][markerId] = currentTcpProjected

                                        # Draw the point in a clean frame
                                        cv2.circle(frameCopy2,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)

                                        # Actualize the column value
                                        yAux += self.distanceBetweenDots

                                    # Actualize the row value
                                    xAux += self.distanceBetweenDots

                                    yAux = 0
                                xAux = 0

                            elif markerId == 2:
                                # Let's iterate over the 6x6 matrix
                                # The location of the translation vectors from the marker 2 to each circle on the whiteboard is calculated using the first t2p as reference
                                for row in range(self.dotsHeight):

                                    for column in range(self.dotsWidth):

                                        # According to the marker 2’s reference frame, the points that share a row will have the same value in y and different values in x (considering the 3.6cm separation between them)
                                        # If you don't get it, check the image where the coordinates of marker 2 are drawn

                                        currentPoint = np.array([x2p + xAux, y2p + yAux, z2p])

                                        currentTcpProjected, _ = cv2.projectPoints(currentPoint, markersPoseSorted[markerId][0], markersPoseSorted[markerId][1], self.cameraMatrix, self.distCoeffs)
                                        currentTcpProjected = np.array(currentTcpProjected).flatten()
                                        tcpointsMatrix[row][column][markerId] = currentTcpProjected

                                        # Draw the point in a clean frame
                                        cv2.circle(frameCopy2,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)

                                        # Actualize the column value
                                        xAux += self.distanceBetweenDots

                                    # Actualize the row value
                                    yAux -= self.distanceBetweenDots # Check where the y-axis is pointing 

                                    xAux = 0
                                yAux = 0

                            elif markerId == 3:
                                # Let's iterate over the 6x6 matrix
                                # The location of the translation vectors from the marker 3 to each circle on the whiteboard is calculated using the first t3p as reference
                                for row in range(self.dotsHeight):

                                    for column in range(self.dotsWidth):

                                        # According to the marker 3’s reference frame, the points that share a row will have the same value in y and different values in x (considering the 3.6cm separation between them)
                                        # If you don't get it, check the image where the coordinates of marker 3 are drawn

                                        currentPoint = np.array([x3p + xAux, y3p + yAux, z3p])

                                        currentTcpProjected, _ = cv2.projectPoints(currentPoint, markersPoseSorted[markerId][0], markersPoseSorted[markerId][1], self.cameraMatrix, self.distCoeffs)
                                        currentTcpProjected = np.array(currentTcpProjected).flatten()
                                        tcpointsMatrix[row][column][markerId] = currentTcpProjected

                                        # Draw the point in a clean frame
                                        cv2.circle(frameCopy2,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)

                                        # Actualize the column value
                                        xAux += self.distanceBetweenDots

                                    # Actualize the row value
                                    yAux -= self.distanceBetweenDots

                                    xAux = 0
                                yAux = 0

                        # Note that I've calculated the first tcpProjected again

                        # tcpointsMatrix.shape ---> (6,6,4,2)

                        # Apply the transformation matrix in the frame
                        # boardWithDotsFrame = cv2.warpPerspective(frameCopy2,transformationMatrix,(self.frameWidth,self.frameHeight))

                        # Compute the average on the second axis
                        tcpointsMatrix = np.mean(tcpointsMatrix, axis=2)
                        # tcpointsMatrix.shape ---> (6,6,2)

                        # Draw the circles in a new frame
                        for row in range(self.dotsHeight):
                            for column in range(self.dotsWidth):

                                # Draw the new matrix on the frame
                                cv2.circle(frameCopy3,(int(tcpointsMatrix[row][column][0]),int(tcpointsMatrix[row][column][1])),5,(255,0,0),thickness=-1)

                        # Apply the transformation matrix in the frame
                        # boardWithDotsAverageFrame = cv2.warpPerspective(frameCopy3,transformationMatrix,(self.frameWidth,self.frameHeight))

                        # The problem now is that since it is constantly detecting the ArUco markers (which can be very unstable depending on the ambient light), 
                        # the matrix of circles on the board appears to move… so if I want to identify a line there, there could be false detections.
                        # Therefore, what I could implement is to continue detecting the points on the board, but the matrix I will use to detect the lines will be an average of each of the points, 
                        # taking x matrices of points.
                        # This way, I think that the detection of the lines will be more stable

                        if countSetOfTcpMatrix < maxSetOfTcpMatrix:
                            setOfTcpointsMatrix.append(tcpointsMatrix)
                            countSetOfTcpMatrix += 1
                        elif countSetOfTcpMatrix == maxSetOfTcpMatrix:

                            # Perform an element-wise operation across every matrix in the set
                            for row in range(self.dotsHeight):
                                for column in range(self.dotsWidth):
                                    auxList = []
                                    for matrix in setOfTcpointsMatrix:
                                        auxList.append(matrix[row][column])
                                    auxMatrix = np.array(auxList)
                                    averageTcpMatrix[row][column] = np.mean(auxMatrix, axis = 0)

                                    # Draw the new matrix on the frame
                                    cv2.circle(frameCopy4,(int(averageTcpMatrix[row][column][0]),int(averageTcpMatrix[row][column][1])),5,(255,0,0),thickness=-1)

                            # Apply the transformation matrix in the frame
                            # boardWithDotsAverageAverage = cv2.warpPerspective(frameCopy4,transformationMatrix,(self.frameWidth,self.frameHeight))

                            # Adapt the perspective of the points in averageTcpMatrix. This way we can use these points in the cropped frame where the whiteboard is isolated from the rest of the image
                            averageTcpMatrixTransformed = cv2.perspectiveTransform(averageTcpMatrix, transformationMatrix)

                            self.boardDetected = True # Update the detection status (average)
                        
                    else:
                        pass
                        # No se están detectando todos los marcadores...

                else:
                    # Display that no ArUco markers were detected
                    markersnotFoundString = 'No se han encontrado marcadores'
                    cv2.putText(frameCopy,markersnotFoundString,(20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)
            
            else:
                print('No se pudo leer un frame.')

        return averageTcpMatrixTransformed, transformationMatrix

    def check_board_detection(self):

        print("\nEjecutando detección del tablero...")

        self.averageTcpMatrixTransformed, self.transformationMatrix = self.detect_board()

        print("Se muestra la detección del tablero. Apretá la letra 'p' para continuar.")

        # Boolean variable for loop control
        runningLoop = True

        # Stop loop event
        def stop_loop(event):
            
            # https://stackoverflow.com/questions/1261875/what-does-nonlocal-do-in-python-3

            nonlocal runningLoop # Allows modifying 'runningLoop' in the outer scope
            runningLoop = False
        
        # Register the event with the 'm' key
        keyboard.on_press_key("p", stop_loop)

        while runningLoop == True:
            # Obtain frame
            frame = self.camera.get_frame()

            if frame is not None:

                boardFrame = cv2.warpPerspective(frame.copy(),self.transformationMatrix,(self.frameWidth,self.frameHeight))

                for row in range(self.dotsHeight):
                    for column in range(self.dotsWidth):
                        # Draw the new matrix on the frame
                        cv2.circle(boardFrame,(int(self.averageTcpMatrixTransformed[row][column][0]),int(self.averageTcpMatrixTransformed[row][column][1])),14,(255,0,0),thickness=-1)
                
                cv2.imshow('Board with dots average average', cv2.resize(boardFrame, (640,480)))

                # Wait a short period to update the window
                cv2.waitKey(1)

        cv2.destroyAllWindows()

    def detect_lines(self, boardFrame, showFrames = True):

        # Get board state
        boardState = self.gameDnbpyLib.get_board_state()

        # Create the frame that will be used to draw the detected lines
        detectedLinesFrame = boardFrame.copy()

        # With this boolean variable, I'll be able to determine if a NEW line was detected
        newLineDetected = False

        # Create the list to store the detected lines with computer vision in dnbpy notation
        detectedLinesList = []

        # Make sure the whiteboard is being detected
        if self.boardDetected == True:

            # The format of the detected lines has to be compatible with the dot and boxes game engine used

            # Now, I'll check each segment between the points to check if there is a line drawn

            # Check horizontal lines
            for row in range(self.dotsHeight):

                # Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [x, y]
                lastDot = [None, None]

                # Define line's type (It could be vertical or horizontal)
                lineType = 'horizontal'

                for column in range(self.dotsWidth):
                    # If it's the first row, do nothing
                    if lastDot == [None, None]:
                        lastDot = [self.averageTcpMatrixTransformed[row][column][0], self.averageTcpMatrixTransformed[row][column][1]]

                    else:
                        # Crop the image to get the particular segment between the two points
                        currentRectangle, topLeftRect, bottomRightRect = utils.crop_image(lastDot, self.averageTcpMatrixTransformed[row][column], lineType, detectedLinesFrame, '1080p', self.boardNumRows, self.boardNumColumns)

                        # Apply Otsu's automatic thresholding
                        threshInv = utils.apply_thresholding(currentRectangle)

                        # Detect if there is a line in the selected segment
                        lineDetection = utils.detect_lines(threshInv)

                        # Draw the rectangle
                        # utils.draw_rectangle(topLeftRect, bottomRightRect, detectedLinesFrame)

                        # Draw each detected line
                        if lineDetection is not None:
                            for i in range(0, len(lineDetection)):
                                l = lineDetection[i][0]
                                cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                                # CurrentRectangle is related with detectedLinesFrames

                        # cv2.imshow('Thresholding', threshInv)
                        # cv2.imshow('Horizontal line detection', currentRectangle)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        
                        # We are checking the current rectangle twice: 
                        # first to detect black lines or other general features, 
                        # and second to detect if there is any red line
                        
                        # We are detecting red color if lineDetection is none
                        if lineDetection is None:
                            # Detect red color (robot's color marker)
                            redColorDetection = utils.red_color_detection(currentRectangle)

                            # Apply Otsu's automatic thresholding with no binary inversion
                            thresh = utils.apply_thresholding_no_inv(redColorDetection)

                            # Detect if there is a line in the selected segment
                            lineDetection = utils.detect_lines(thresh)

                            # Draw each detected line
                            if lineDetection is not None:
                                for i in range(0, len(lineDetection)):
                                    l = lineDetection[i][0]
                                    cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                                    # CurrentRectangle is related with detectedLinesFrames
                        
                            # cv2.imshow('Thresholding', thresh)
                            # cv2.imshow('Horizontal line detection', currentRectangle)
                            # cv2.waitKey(0)
                            # cv2.destroyAllWindows()

                        # Update the past point
                        lastDot = [self.averageTcpMatrixTransformed[row][column][0], self.averageTcpMatrixTransformed[row][column][1]]

                        # Get the dnbpy contention of the detected line
                        if lineDetection is not None:
                            detectedHorizontalLineDnbpyConv = int(self.horizontalLinesMatrix[row][column-1])
                            # print(f'{detectedHorizontalLineDnbpyConv} - {lineType}')
                            # [column-1] because, in the first iteration of this loop, lastDot = [None, None]

                            # Update the line detection status if necessary
                            if boardState[detectedHorizontalLineDnbpyConv] == 0:
                                # print('Nueva línea detectada:', str(detectedHorizontalLineDnbpyConv))
                                detectedLinesList.append(detectedHorizontalLineDnbpyConv)
                                newLineDetected = True # A NEW line was detected

            # Check vertical lines
            for column in range(self.dotsWidth):

                # Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [x, y]
                lastDot = [None, None]

                # Define line's type (It could be vertical or horizontal)
                lineType = 'vertical'

                for row in range(self.dotsHeight):
                    # If it's the first row, do nothing
                    if lastDot == [None, None]:
                        lastDot = [self.averageTcpMatrixTransformed[row][column][0], self.averageTcpMatrixTransformed[row][column][1]]
                    else:
                        # Crop the image to get the particular segment between the two points
                        currentRectangle, topLeftRect, bottomRightRect = utils.crop_image(lastDot, self.averageTcpMatrixTransformed[row][column], lineType, detectedLinesFrame, '1080p', self.boardNumRows, self.boardNumColumns)
                        
                        # Apply Otsu's automatic thresholding
                        threshInv = utils.apply_thresholding(currentRectangle)

                        # Detect if there is a line in the selected segment
                        lineDetection = utils.detect_lines(threshInv)

                        # Draw the rectangle
                        # utils.draw_rectangle(topLeftRect, bottomRightRect, detectedLinesFrame)

                        # Draw each detected line
                        if lineDetection is not None:
                            for i in range(0, len(lineDetection)):
                                l = lineDetection[i][0]
                                cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                                # CurrentRectangle is related with detectedLinesFrames

                        # cv2.imshow('Vertical line detection', currentRectangle)
                        # cv2.imshow('Thresholding', threshInv)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()

                        # We are checking the current rectangle twice: 
                        # first to detect black lines or other general features, 
                        # and second to detect if there is any red line
                        
                        # We are detecting red color if lineDetection is none
                        if lineDetection is None:
                            # Detect red color (robot's color marker)
                            redColorDetection = utils.red_color_detection(currentRectangle)

                            # Apply Otsu's automatic thresholding with no binary inversion
                            thresh = utils.apply_thresholding_no_inv(redColorDetection)

                            # Detect if there is a line in the selected segment
                            lineDetection = utils.detect_lines(thresh)

                            # Draw each detected line
                            if lineDetection is not None:
                                for i in range(0, len(lineDetection)):
                                    l = lineDetection[i][0]
                                    cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                                    # CurrentRectangle is related with detectedLinesFrames
                        
                            # cv2.imshow('Thresholding', thresh)
                            # cv2.imshow('Horizontal line detection', currentRectangle)
                            # cv2.waitKey(0)
                            # cv2.destroyAllWindows()

                        # Update the past point
                        lastDot = [self.averageTcpMatrixTransformed[row][column][0], self.averageTcpMatrixTransformed[row][column][1]]
                        
                        # Get the dnbpy contention of the detected line
                        if lineDetection is not None:
                            detectedVerticalLineDnbpyConv = int(self.verticalLinesMatrix[row-1][column])

                            # print(f'{detectedVerticalLineDnbpyConv} - {lineType}')
                            # [row-1] because, in the first iteration of this loop, lastDot = [None, None]
                            
                            # Update the line detection status if necessary
                            if boardState[detectedVerticalLineDnbpyConv] == 0:
                                # print('Nueva línea detectada:', str(detectedVerticalLineDnbpyConv))
                                detectedLinesList.append(detectedVerticalLineDnbpyConv)
                                newLineDetected = True # A NEW line was detected

            if showFrames == True:
                # Display the complete frame
                cv2.imshow('Detected lines frame', cv2.resize(detectedLinesFrame, (640,480)))

                # Wait a short period to update the window
                cv2.waitKey(1)
        
        return detectedLinesList, newLineDetected

    def check_setup(self):

        print('\nEs necesario chequear el setup. Si ves que todos los marcadores se detectan sin inconvenientes no tenés que hacer nada. De lo contrario vas a tener que solucionarlo.')
        print('Es conveniente que la cámara esté lo más cerca posible del tablero.')
        print('Si la detección de los marcadores es buena, apretá la letra "m". Además, asegurate de que no se modifique la iluminación ni la posición de la cámara a lo largo de la partida.')

        # Boolean variable for loop control
        runningLoop = True

        # Stop loop event
        def stop_loop(event):
            
            # https://stackoverflow.com/questions/1261875/what-does-nonlocal-do-in-python-3

            nonlocal runningLoop # Allows modifying 'runningLoop' in the outer scope
            runningLoop = False
        
        # Register the event with the 'm' key
        keyboard.on_press_key("m", stop_loop)

        while runningLoop == True:

            # Obtain frame
            frame = self.camera.get_frame()

            if frame is not None:
                
                # Copy the frame
                frameCopy = frame.copy()

                # Convert to gray scale
                gray = cv2.cvtColor(frameCopy, cv2.COLOR_BGR2GRAY)

                # Detect ArUco markers in the video frame
                corners, ids, rejected = self.arucoDetector.detectMarkers(gray)

                # Draw ArUco markers detected
                cv2.aruco.drawDetectedMarkers(frameCopy, corners)

                # If at least one ArUco markers is detected, then execute the following code; otherwise (None cases) don't execute
                if ids is not None:

                    # print(len(corners)) # n
                    # print(corners[0].shape) # (n,1,4,2) -> n is in the tuple layer (in this case, n = 0 --> corners[0].shape)
                    # print(ids.shape) # (n,1)

                    # Flatten the ArUco IDs and make it a list
                    ids = ids.flatten().tolist()
                    # print(ids.shape) # (n,)

                    # Display how many ArUco markers are being detected
                    markersFoundString = f'{len(ids)} marcadores encontrados.'
                    cv2.putText(frameCopy,markersFoundString,(60,60), cv2.FONT_HERSHEY_DUPLEX, 2,(0,255,0),2)
            
                cv2.imshow('Frame with markers', cv2.resize(frameCopy, (640, 480)))
                cv2.waitKey(1)
        
        cv2.destroyAllWindows()

        # After typing "m"...
        print('Bien, iluminación confirmada.')

    def get_current_frame_transformed(self):
        
        frame = self.camera.get_frame()

        if frame is not None:

            boardFrame = cv2.warpPerspective(frame.copy(),self.transformationMatrix,(self.frameWidth,self.frameHeight))

            boardFrameCopy1 = boardFrame.copy()

            for row in range(self.dotsHeight):
                for column in range(self.dotsWidth):
                    # Draw the new matrix on the frame
                    cv2.circle(boardFrameCopy1,(int(self.averageTcpMatrixTransformed[row][column][0]),int(self.averageTcpMatrixTransformed[row][column][1])),14,(255,0,0),thickness=-1)

            cv2.imshow('Game board', cv2.resize(boardFrame, (640,480)))
            cv2.imshow('Board with dots average average', cv2.resize(boardFrameCopy1, (640,480)))

            if self.previousFrame is not None:
                cv2.imshow('Previous frame', cv2.resize(self.previousFrame, (640,480)))

            cv2.waitKey(1)

            return boardFrame

    def is_whiteboard_empty(self):

        # This function only detects if a line was drawn before the game started. However, it won't detect any scribbles

        # Get board state
        boardState = self.gameDnbpyLib.get_board_state()

        if sum(boardState) == 0:

            while True:

                print('\nChequeando pizarra...')

                boardFrame = self.get_current_frame_transformed()

                # Detect lines
                detectedLinesList, newLineDetected = self.detect_lines(boardFrame=boardFrame)

                if newLineDetected == False:
                    print('¡Excelente, la pizarra está vacía!')
                    print('\n¡El juego ya puede comenzar!')

                    # Save the current boardFrame as the previous frame
                    self.previousFrame = boardFrame
                    # Ensure this frame contains only the game board and no additional elements
                    # This is crucial since we don't have a previous image to compare with, as it is defined as None initially

                    # The whiteboard is empty, so we can quit the loop
                    break

                else:
                    print('Se detectaron líneas en la pizzarra. Borralas e ingresá la letra "r" cuando ya lo hayas hecho.')
                    user_input2 = input('>> ')

                    if user_input2 == 'r':
                        print('Bien, volviendo a chequear la pizarra...')
                    else:
                        print('Ah, bueno, te hacés el vivo. Vuelvo a chequear la pizarra, me da igual si la borraste o no.')

        else:
            print('Este método solo es útil al principio, no tiene sentido usarlo con el juego ya comenzado.')
    
    def compare_frames(self, possibleCurrentFrame):
        img1 = cv2.cvtColor(self.previousFrame.copy(), cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(possibleCurrentFrame.copy(), cv2.COLOR_BGR2GRAY)

        # Ensure the images have the same shape
        if img1.shape == img2.shape:
            # Differenciate the frames
            difference = cv2.absdiff(img1, img2)

            # cv2.imshow('Difference', cv2.resize(difference, (640,480)))
            # cv2.imshow('1', cv2.resize(img1, (640,480)))
            # cv2.imshow('2', cv2.resize(img2, (640,480)))
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        else:
            print("Las imágenes tienen dimensiones diferentes.")

        # Normalized Euclidean Distance (NED)
        totalPixels = 1080*1920
        normEuclideanDistance = (np.linalg.norm(difference) / totalPixels)*100
        print(f"NED: {normEuclideanDistance}")

        return normEuclideanDistance

    def is_diff_error_ok(self,boardFrame):
        diffFrameError = self.compare_frames(boardFrame)

        return diffFrameError <= self.AcceptedDiffFrameError

    def play(self):

        print('\n------------------------------------------')
        print('Que comience el juego...')

        # Continue the loop as long as the game isn't over
        while not self.has_finished():

            # Get the current player
            currentPlayer = self.gameDnbpyLib.get_current_player()

            # The detection will be one line at a time, at least in these early versions of the project

            # This variable will be used in a while loop that keeps iterating until a line is drawn. The user will manually confirm when the line is drawn
            runningLoop = True

            # If the player makes the move...
            if currentPlayer == self.players[0]:
                print(f'\nTurno de {currentPlayer}.')
                
                # Continue running this while loop until a line drawn by the player is detected
                while runningLoop == True:

                    # print('Cuando termines de dibujar la línea, apretá "m".')
                    # keyboard.wait('m')

                    # Get the current frame
                    boardFrame = self.get_current_frame_transformed()

                    if self.is_diff_error_ok(boardFrame): # If true...

                        # Detect lines
                        detectedLinesList, newLineDetected = self.detect_lines(boardFrame=boardFrame)

                        # If a new line was detected...
                        if newLineDetected == True:

                            # Update the loop control variable to manage loop execution
                            runningLoop = False

                            # A new line was detected, so it has to be the last one in the list
                            playerMove = detectedLinesList[-1]

                            # Here is an issue to consider: If I drew three lines and then pressed ‘m,’, only the last one will be taken into account. 
                            # But of course, this breaks the entire logic because the list would have two other lines that are supposedly already drawn, but in reality, they are not
                            # So it would be: I draw a line, press ‘p’, draw a line, press ‘p’, and so on

                            # Make the move in the dnb engine
                            self.gameDnbpyLib.select_edge(playerMove, currentPlayer)

                            print(f'Movimiento de {self.players[0]}: {playerMove}.')
                            
                            # The camera increases the brightness significantly after a sudden change (e.g., when I remove my hand from the camera after drawing a line)
                            # Therefore, it is advisable to wait a few seconds to allow the camera to stabilize again and then take another frame
                            sleep(0.5)
                            
                            boardFrame = self.get_current_frame_transformed()

                            while not self.is_diff_error_ok(boardFrame):
                                boardFrame = self.get_current_frame_transformed()

                            # Update the previous frame
                            self.previousFrame = boardFrame

                    #     else:
                    #         print('No se detectó que hayas dibujado alguna línea.')
                        
                    # else:
                    #     print('Se detectaron interferencias sobre la pizarra...')

            # If the robot makes the move...
            elif currentPlayer == self.players[1]:
                print(f'\nTurno del robot.')

                # Get the move for the robot
                robotMove = self.aiAgent.select_edge(self.gameDnbpyLib.get_board_state(), self.gameDnbpyLib.get_score(currentPlayer), self.gameDnbpyLib.get_score(self.players[0]))
                print(f'{robotMove} (Auxiliar)')

                # I need to know whether it's a vertical or horizontal line
                # Based on the robot's movement, I have to find between which two points the line should be drawn
                pointsIndexforLine, lineType2 = utils.dnb_conv_to_points_index(robotMove, self.horizontalLinesMatrix, self.verticalLinesMatrix, 
                                                                               self.pointsHorizontalLinesMatrix, self.pointsVerticalLinesMatrix)
                # Indices of both points, which are related to the 6x6 distance matrix

                # Use those indices in the 6x6 distance matrix from the base frame of the robot to the circles on the whiteboard to get the initial and final points of the line
                initialPoint = self.tbpointsMatrix[int(pointsIndexforLine[0][0][0])][int(pointsIndexforLine[0][0][1])]
                finalPoint = self.tbpointsMatrix[int(pointsIndexforLine[0][1][0])][int(pointsIndexforLine[0][1][1])]

                # Points = [x, y, z, pitchAngle]
                initialPoint = list(initialPoint)
                finalPoint = list(finalPoint)
                
                print(f'Punto inicial: {initialPoint}')
                print(f'Punto final: {finalPoint}')
                print(lineType2)

                # Send the instructions to the motors via serial communication with the ESP32
                open_loop_traj_control.make_robot_play(initialPoint, finalPoint)

                # Continue running this while loop until a line drawn by the robot is detected and it must be the correct one
                while runningLoop == True:
                    # print('Cuando el robot haya terminado de dibujar la línea, apretá "m".')
                    # keyboard.wait('m')

                    # Get the current frame
                    boardFrame = self.get_current_frame_transformed()

                    if self.is_diff_error_ok(boardFrame):

                        # Detect lines
                        detectedLinesList, newLineDetected = self.detect_lines(boardFrame=boardFrame)

                        if newLineDetected == True:
                            if detectedLinesList[-1] == robotMove:

                                # Update the loop control variable to manage loop execution
                                runningLoop = False

                                # Make the move in the dnb engine
                                self.gameDnbpyLib.select_edge(robotMove, currentPlayer)

                                print(f'Movimiento del {self.players[1].lower()}: {robotMove}.')

                                # The camera increases the brightness significantly after a sudden change (e.g., when I remove my hand from the camera after drawing a line)
                                # Therefore, it is advisable to wait a few seconds to allow the camera to stabilize again
                                sleep(0.5)

                                boardFrame = self.get_current_frame_transformed()

                                while not self.is_diff_error_ok(boardFrame):
                                    boardFrame = self.get_current_frame_transformed()

                                # Update the previous frame
                                self.previousFrame = boardFrame

                            else:
                                print(f'No se dibujó la línea en el casillero correcto ({detectedLinesList[-1]}). Bórrela de la pizarra.')
                    #     else:
                    #         print('No se detectó que el robot haya dibujado alguna línea.')
                    # else:
                    #     print('Se detectaron interferencias sobre la pizarra...')

            # Print the score in the terminal
            self.show_score()
        
        # If the match has ended...
        if self.has_finished() == True:
            print('----------------------------------------')
            print('La partida terminó.\n')

            # Get individual scores
            scorePlayer = self.gameDnbpyLib.get_score(self.players[0])
            scoreRobot = self.gameDnbpyLib.get_score(self.players[1])

            print('Resultados:')
            self.show_score()

            if scorePlayer > scoreRobot:
                print(f'¡Felicitaciones {self.players[0]}, eres el ganador!')
            elif scoreRobot > scorePlayer:
                print(f'Ganó el robot... La humanidad está perdida...')
            elif scoreRobot == scorePlayer:
                print('¡Es un empate!')

    def show_score(self):
        print(f'\n{self.players[0]}: {self.gameDnbpyLib.get_score(self.players[0])}, {self.players[1]}: {self.gameDnbpyLib.get_score(self.players[1])}')
















if __name__ == '__main__':
    pass