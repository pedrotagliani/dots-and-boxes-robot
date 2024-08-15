# Import libraries
import numpy as np
import cv2
import pickle
import keyboard
import dnbpy as dnb
from interpolator import simple_interpolator
from python_esp32_communication import serial_communication_for_servos
import threading




from modules import camera






#############################################################################
################## Variable definition and main loop setup ##################
#############################################################################

# Load the data from the pickle file
with open('config/camera_config.pckl', 'rb') as file:
	loadedData = pickle.load(file)

# Camera parameters obtained from calibration
cameraMatrix = loadedData[0]
distCoeffs = loadedData[1]

# Define the ArUco detector
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams =  cv2.aruco.DetectorParameters()
# arucoParams.minMarkerDistanceRate = 0.03
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Capture webcam
videoSource = 0
cap = camera.myCamera(videoSource)

# Obtain width and height of the video capture
frameWidth = cap.width # 640
frameHeight = cap.height # 480

# Define variables related to the game
dotsWidth = 6
dotsHeight = 6
distanceBetweenDots = 3.6 # Distance in centimetre
boardNumColumns = dotsWidth - 1
boardNumRows = dotsHeight - 1
boardTotalLines = dotsHeight*boardNumRows + dotsWidth*boardNumColumns
# boardNumVerticalOrHorizontalLines = dotsHeight*boardNumRows + dotsWidth*boardNumColumns
# print(boardTotalLines) # 6x6 --> 60 lines
# print(boardNumVerticalOrHorizontalLines)

# Variable to check if the four markers where are being found in the whiteboard
squareFound = False

# Variables to store the tcp matrices when the square is being detected:
setOfTcpointsMatrix = []
countSetOfTcpMatrix = 0
maxSetOfTcpMatrix = 100
firstAverageMatrixFound = False

# An array that stores the values obtained from performing element-wise operations on matrices
averageTcpMatrix = np.zeros((dotsHeight,dotsWidth,2), dtype=np.float32)

# Auxiliar variables
firstCheck = False # The first detection of the board to look for lines
emptyBoard = False # If the board is empty, then emptyBoard = True. Otherwise, emptyBoard = False
gameBegan = False # The game has begun
alreadyDenied = False # An auxiliary boolean that helps to avoid entering the same loop again
oneTime = True # An auxiliary boolean that helps to avoid entering the same if statement again
robotMoveGenerated = False # Variable to record whether the robot movement was generated




########################################################################
################## Function definitions for main loop ##################
########################################################################

def crop_image(point1, point2, lineType):
	if lineType == 'horizontal':
	
		# Define corners for the new rectangle (considering a padding in the y axis)
		padding = (frameHeight/boardNumRows)*0.1
		# Be careful with the padding, if it surpass the borders of the image, an error will raise in gray

		# Select two opposite vertices of the rectangle
		topLeftRectangle = [point1[0], point1[1] - padding]
		bottomRightRectangle = [point2[0], point2[1] + padding]

		# https://stackoverflow.com/questions/9084609/how-to-copy-a-image-region-using-opencv-in-python
		# https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python
	
	elif lineType == 'vertical':
		# Define corners for the new rectangle (considering a padding in the x axis)
		padding = (frameWidth/boardNumRows)*0.1
		# Be careful with the padding, if it surpass the borders of the image, an error will raise in gray

		# Select two opposite vertices of the rectangle
		topLeftRectangle = [point1[0] - padding, point1[1]]
		bottomRightRectangle = [point2[0] + padding, point2[1]]

		# https://stackoverflow.com/questions/9084609/how-to-copy-a-image-region-using-opencv-in-python
		# https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python

	# Crop the image using numpy slicing
	# currentRectangle = transformation1[int(topLeftRectangle[1]):int(bottomRightRectangle[1]), int(topLeftRectangle[0]):int(bottomRightRectangle[0])].copy()
	currentRectangle = transformation1[int(topLeftRectangle[1]):int(bottomRightRectangle[1]), int(topLeftRectangle[0]):int(bottomRightRectangle[0])]

	# Return the cropped image
	return currentRectangle, topLeftRectangle, bottomRightRectangle

def draw_rectangle(topLeftRectangle, bottomRightRectangle):
	# Draw the rectangle in the image
	cv2.rectangle(transformation1, (int(topLeftRectangle[0]), int(topLeftRectangle[1])), (int(bottomRightRectangle[0]), int(bottomRightRectangle[1])), (255, 0, 0), 1)
	# I shouldn't draw the rectangle before the line detection

def apply_thresholding(cropedImage):
	# Convert to grayscale
	gray = cv2.cvtColor(cropedImage, cv2.COLOR_BGR2GRAY)

	# Apply Otsu's automatic thresholding
	(T, threshInv) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

	# Return the thresholded image
	return threshInv

def detect_lines(thresholdedImage):
	# https://stackoverflow.com/questions/41329665/linesegmentdetector-in-opencv-3-with-python
	# lineDetector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_ADV)
	# lines = lineDetector.detect(gray)[0]
	# draw = lineDetector.drawSegments(gray, lines)

	# lines = cv2.HoughLines(threshInv, 1, np.pi/180, 80, None, 0, 0)

	# if lines is not None:
	# 	for i in range(0, len(lines)):
	# 		rho = lines[i][0][0]
	# 		theta = lines[i][0][1]
	# 		a = np.cos(theta)
	# 		b = np.sin(theta)
	# 		x0 = a * rho
	# 		y0 = b * rho
	# 		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
	# 		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
	# 		cv2.line(currentRectangle, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

	# https://docs.opencv.org/4.10.0/dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a
	# https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html

	# Apply the Probabilistic Line Transform to detect if a line was drawn
	# linesP = cv2.HoughLinesP(thresholdedImage, 1, np.pi / 180, 40, None, 50, 10)
	linesP = cv2.HoughLinesP(thresholdedImage, rho=1, theta=np.pi/180, threshold=40, minLineLength=50, maxLineGap=15)


	# linesP = Output vector of lines. Each line is represented by a 4-element vector (x1,y1,x2,y2),
	# where (x1,y1) and (x2,y2) are the ending points of each detected line segment

	return linesP




###########################################################
##################### Keyboard events #####################
###########################################################

# Define variables for keyboard events
nextStep = False # Variable for enabling the detection of the lines
startGame = False # Variable for starting the game

def on_key_event(e):
	global nextStep
	global startGame
	# print("Key pressed:", e.name)
	if e.name == 'p':
		nextStep = True
	elif e.name == 'm':
		startGame = True

# Register a callback function to be called on a key press event
keyboard.on_press(on_key_event)




###################################################################################################################
###################### Game configuration involving initialization, variables, and functions ######################
###################################################################################################################

# Set the players
players = ['Jugador', 'Robot']
player = players[0]
robot = players[1]

# Define the size of the board
boardSize = (boardNumRows, boardNumColumns)

# AI agent
minimaxAgent = dnb.Level3MinimaxPolicy(boardSize, depth=2, update_alpha=True)

# Create a new game instances
game = dnb.Game(boardSize, players)

# Change the current player (default value is 0)
game._current_player = 0

# Create the list to store the detected lines with computer vision in dnbpy notation:
linesDetectionStatus = []

# The dnbpy library has a specific method for storing board lines, which we will save in a numpy array. This approach allows us to match detected 
# lines in specific [row][column] positions with the dbd matrix, enabling us to register moves in the game
def get_dnb_matrix_convention(boardRows, boardColumns, totalLines):

	# Matrices for storing the numbers
	horizontalLines = np.zeros(shape=(boardRows + 1, boardColumns)) # (6,5)
	verticalLines = np.zeros(shape=(boardRows, boardColumns + 1)) # (5,6)

	# Maximum number of lines per row
	ver = boardColumns +  1
	hor = boardColumns

	# Auxiliar variables
	aux1 = 1
	aux2 = 1
	aux3 = 0
	aux4 = 0
	aux5 = 0
	aux6 = 0

	# Fill both matrices using the dnbpy convention
	# For example, if the board is 2x2 (rows x columns), the horizontal lines array would be: [[0, 1], [5, 6], [10, 11]]
	# and the vertical lines array would be: [[2, 3, 4], [7, 8, 9]]
	for j in range(0,totalLines):
		if aux1 <= hor:
			horizontalLines[aux3][aux4] = j
			aux1 += 1
			aux4 +=1
			if aux4 == hor:
				aux4 = 0

		elif aux2 <= ver and aux1 == hor + 1:
			verticalLines[aux5][aux6] = j
			aux2 += 1
			aux6 += 1
			if aux6 == ver:
				aux6 = 0
		
		if aux2 == ver + 1 and aux1 == hor + 1:
			aux1 = 1
			aux2 = 1
			aux3 += 1
			aux5 += 1

	# Return the filled arrays
	return horizontalLines, verticalLines

# Matrices for storing the dnbpy convention
horizontalLinesMatrix, verticalLinesMatrix = get_dnb_matrix_convention(boardNumRows, boardNumColumns, boardTotalLines)

# Using the dnb convention, determine the points for drawing each vertical and horizontal line and store them in two matrices
def dnb_conv_to_matrix_points(boardRows, boardColumns):
	# Matrices for storing the points
	pointsOfHorizontalLines = np.zeros(shape=(boardRows + 1, boardColumns,2,2)) # (6,5)
	pointsOfVerticalLines = np.zeros(shape=(boardRows, boardColumns + 1,2,2)) # (5,6)

	# For horizontal lines
	for row in range(dotsHeight):

		# Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [row, column]
		lastDot = [None, None]

		for column in range(dotsWidth):
			# If it's the first row, do nothing
			if lastDot == [None, None]:
				lastDot = [row,column]
			else:
				# Initially, [row, column] = [0,0] and lastDot = [None,None], so it is updated to lastDot = [0,0].
				# In the next iteration, when lastDot = [0,0] (not equal to [None, None]) and [row, column] = [0,1], this line should be stored in pointsOfHorizontalLines[0,0] because it represents the first line.
				# Therefore, we can generalize this to pointsOfHorizontalLines[row, column-1].

				pointsOfHorizontalLines[row,column-1] = np.array([np.array(lastDot),np.array([row,column])])

				# Update the past point
				lastDot = [row,column]

	# For vertical lines
	for column in range(dotsWidth):

		# Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [row, column]
		lastDot = [None, None]

		for row in range(dotsHeight):
			# If it's the first row, do nothing
			if lastDot == [None, None]:
				lastDot = [row,column]
			else:
				# Initially, [row, column] = [0,0] and lastDot = [None,None], so it is updated to lastDot = [0,0].
				# In the next iteration, when lastDot = [0,0] (not equal to [None, None]) and [row, column] = [1,0], this line should be stored in pointsOfHorizontalLines[0,0] because it represents the first line.
				# Therefore, we can generalize this to pointsOfHorizontalLines[row-1, column].

				pointsOfVerticalLines[row-1,column] = np.array([np.array(lastDot),np.array([row,column])])

				# Update the past point
				lastDot = [row,column]
	
	return pointsOfHorizontalLines, pointsOfVerticalLines

# Matrices for storing the pair of points that are involved in the creation of a particular line
pointsHorizontalLinesMatrix, pointsVerticalLinesMatrix = dnb_conv_to_matrix_points(boardNumRows, boardNumColumns)

# From the robot movement in dnb convention, determine between which two points the line should be drawn
def dnb_conv_to_points_index(robotMovement):
	# Determine if the robot movement is inside of the matrix for horizontal or vertical lines, which are also in the dnb convention
	if robotMovement in horizontalLinesMatrix:
		# Retrieve the index of a specific value stored in the array
		index = np.where(horizontalLinesMatrix == robotMovement)

		# pointsHorizontalLinesMatrix and horizontalLinesMatrix represent the same lines, so the indices in both matrices correspond to the same lines
		pointsIndex = pointsHorizontalLinesMatrix[index[0],index[1]]

		line = 'Horizontal'

	elif robotMovement in verticalLinesMatrix:
		# Retrieve the index of a specific value stored in the array
		index = np.where(verticalLinesMatrix == robotMovement)

		# pointsVerticalLinesMatrix and VerticalLinesMatrix represent the same lines, so the indices in both matrices correspond to the same lines
		pointsIndex = pointsVerticalLinesMatrix[index[0],index[1]]

		line = 'Vertical'
	
	# Return the indeces of botch points that are used to draw the line
	return pointsIndex, line
	# These point indices correspond to the 6x6 distance matrix from the base frame of the robot to each dot on the whiteboard










#################################################################################################################################
################### Matrix for storing all the distances from the robotic arm to the points on the whiteboard ###################
#################################################################################################################################

# The width of the wood is 21.4cm
# The length of the board (excluding the black frames) is 33cm.
# To center it, there is 5.8cm on each side.

# Distances from the robot base frame to the first point of the board: xbp = 19.4cm ybp = -9.5cm zbp = 0 (This needs to be checked)
# ALL THESE MEASUREMENTS STILL NEED TO BE CONFIRMED."

# Distance from the robot to the first point on the whiteboard (take into account the direction in which the axis of the base frame points)
# If not clear yet, check the image v2_Robot to circles.png
xbp1 = 19
ybp1 = -8.2
zbp = 0.2

# I would say that the distances, as such, are not the real ones. This is due to the accumulated error 
# (the angles that the servos are asked to turn have a certain error, and the distances of the links as well).

# Matrix for storing the translation vectors from the base frame of the robot to the points on the whiteboard
tbpointsMatrix = np.zeros((dotsHeight,dotsWidth,3), dtype=np.float32)

# Auxiliar variables
xAux2 = 0
yAux2 = 0

# Let's iterate over the 6x6 matrix
# The location of the translation vectors from the base frame of the robot to each circle on the whiteboard is calculated using the first tbp1 as reference
for row in range(dotsHeight):

    for column in range(dotsWidth):

        # According to the base reference frame of the robot, the points that share a row will have the same value in x and different values in y (considering the 3.6cm separation between them)
        # If you don't get it, check the image v2_Robot to circles.png

        tbpointsMatrix[row,column] = np.array([xbp1 + xAux2, ybp1 + yAux2, zbp])

        # Actualize the column value
        yAux2 += distanceBetweenDots

    # Actualize the row value
    xAux2 += distanceBetweenDots

    yAux2 = 0
xAux2 = 0

# print(tbpointsMatrix)





#######################################################################
############################## Threading ##############################
#######################################################################

def thread_serial_communication(interpolationPoints):
	# Create the thread
	t1 = threading.Thread(target=serial_communication_for_servos, args=(interpolationPoints,), daemon=True)
	# If daemon = True, the program will exit once the main thread completes, regardless of the other threads
	# So, if this thread is executing and I press the 'q' key in the main program, both the main thread and this thread will finish execution

	# Start the thread
	t1.start()





###################################################################
############################ Main Loop ############################
###################################################################

# Continue the loop as long as the game isn't over
while not game.is_finished():

	# Obtain a frame
	frame = cap.get_frame()

	if frame is not None:

		# Auxiliar frames
		originalFrame = frame.copy()
		originalFrame2 = frame.copy()
		originalFrame3 = frame.copy()


		# Convert to gray scale
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


		# Detect ArUco markers in the video frame
		corners, ids, rejected = arucoDetector.detectMarkers(gray)

		# Draw ArUco markers detected
		cv2.aruco.drawDetectedMarkers(frame, corners)

		# If at least one ArUco markers is detected, then execute the following code; otherwise (None cases) don't execute
		if len(corners) > 0:

			# print(len(corners)) # n
			# print(corners[0].shape) # (n,1,4,2) -> n is in the tuple layer (in this case, n = 0 --> corners[0].shape)
			# print(ids.shape) # (n,1)

			# Flatten the ArUco IDs and make it a list
			ids = ids.flatten().tolist()
			# print(ids.shape) # (n,)

			# Display how many ArUco markers are being detected
			markersFoundString= f'{len(ids)} marcadores encontrados.'
			cv2.putText(frame,markersFoundString,(20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)

			# Array to store the left corner from all the markers
			leftCornersMarkers=[]

			# Here, I'll store the pose of the marker 0 respect to the camera frame
			poseId0 = [None, None]

			# Iterate over every detected ArUco marker
			for markerCorner, markerId in zip(corners, ids):

				# Subpixel corner detection
				criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
				markerCornerUnmodified = cv2.cornerSubPix(gray, markerCorner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

				# Reshape the array
				markerCorner2 = markerCornerUnmodified.reshape((4, 2))

				# Classify each corner of the marker
				(topLeft, topRight, bottomRight, bottomLeft) = markerCorner2

				# Append the top left corner of the ArUco marker in the previously created array
				leftCornersMarkers.append([topLeft[0],topLeft[1]])

				# Draw the top left corner and display the id number of the marker
				cv2.circle(frame,(int(topLeft[0]),int(topLeft[1])),5,(0,0,255),thickness=-1)
				cv2.putText(frame, str(markerId), (int(bottomRight[0]+10), int(bottomRight[1]+10)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
				
				# Size of the marker in centimetre
				markerSizeInCM = 3.8

				# Find the pose of each marker respect to the camera
				rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCornerUnmodified, markerSizeInCM, cameraMatrix, distCoeffs)
				
				# Flatten the position from the camera to the marker
				tvec = tvec.flatten()
			
				# Draw frame axis:
				cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, length=8, thickness=4)

				# I am only interested in save the pose of the marker with index 0 (Later, I'll relate it with every circle of the real matrix)
				if markerId == 0:
					poseId0[0] = rvec
					poseId0[1] = tvec

			# If the IDs 0, 1, 2 and 3 are being detected, then execute the if statement
			if (0 in ids) and (1 in ids) and (2 in ids) and (3 in ids): # This is done to avoid false detections (for example, [2,1,3,38])
				
				# Overlay to draw the polygon
				overlay = frame.copy()

				# print(ids)

				# List to store the top left marker corner sorted by ID number
				leftMarkerCornersSorted = [None, None, None, None]

				for i in range(4): #0,1,2,3
					index = ids.index(i) # if ids = [3,1,2] --> i = 3 --> index = 0
					leftMarkerCornersSorted[i] = leftCornersMarkers[index]

				# Convert from list to array so I can use cv2.fillPolly
				leftMarkerCornersSorted = np.array(leftMarkerCornersSorted)

				# https://stackoverflow.com/questions/17241830/opencv-polylines-function-in-python-throws-exception
				cv2.fillPoly(overlay, pts = np.int32([leftMarkerCornersSorted]), color=(255,215,0))
				
				# Blend the overlay and the original frame with cv2.addWeighted
				alpha = 0.4  # Transparency factor
				beta = 1 - alpha # Transparency factor
				gamma = 0 # Transparency factor
				frame = cv2.addWeighted(frame, alpha, overlay, beta, gamma)

				squareFound = True
			else:
				squareFound = False
			
			# The current objective is to center the whiteboard for optimal game visualization
			if squareFound == True:

				# Destination coordinates sorted by top left, top right, bottom left, bottom right:
				ptsDestination = np.float32([[0,0],[frameWidth,0],[0,frameHeight],[frameWidth,frameHeight]])

				# The source points will be static. In other words, they'll ve related with the same ArUco Markers
				# It doesn't matter where the camera is placed as long as the markers are correctly detected
				# It's important to keep in mind that the top left corner of the markers are already sorted by IDs in the array called leftMarkerCornersSorted

				# Source coordinates sorted by top left, top right, bottom left, bottom right:
				ptsSource = np.float32([leftMarkerCornersSorted[0], leftMarkerCornersSorted[3], leftMarkerCornersSorted[1], leftMarkerCornersSorted[2]])
				# If you take a look at the image with the marker IDs displayed, it's easy to to find the correct order in ptsSource

				# Obtain the transfomation matrix:
				transformationMatrix = cv2.getPerspectiveTransform(ptsSource,ptsDestination)
				
				# Apply the transformation matrix in the original frame
				transformation1 = cv2.warpPerspective(originalFrame,transformationMatrix,(frameWidth,frameHeight))

				# # Display the transformed frame
				# cv2.imshow('Juego',transformation1)


				# Now, I should find where the dots are placed on the whiteboard
				# The markers are fixed (they are always in the same place), so the distance from every marker to the circles on the whiteboard will always be the same
				# I'm gonna reference every point to the maker with ID 0
				# If I take the distance from that marker to only one circle, I can calculate the rest because they are all separated by approximately 3.6cm

				# Distance from marker with ID 0 to the first circle ---> (-0.7cm, 6.1cm, 0cm)
				x0p = -0.7
				y0p = 6.1
				z0p = 0

				# I know the pose of this marker with respect to the camera. Hence I can construct the homogeneous transformation matrix that maps points from the marker’s coordinate frame to the camera’s coordinate frame

				# The rotation obtained is expressed in Rodrigues’ angles; therefore, I will convert it to a rotation matrix (camera to 0)
				Rc0 = cv2.Rodrigues(poseId0[0])[0] # It returns a tuple with a 3x3 matrix inside corresponding to the rotation matrix

				# The translation vector from the camera to the marker 0 was already obtained (camera to 0)
				tc0 = np.array(poseId0[1]).flatten()

				# Construct the HTM:
				# Ac0 = np.array([[Rc0[0][0], Rc0[0][1], Rc0[0,2], tc0[0]],
				# 				[Rc0[1][0], Rc0[1][1], Rc0[1,2], tc0[1]],
				# 				[Rc0[2][0], Rc0[2][1], Rc0[2,2], tc0[2]],
				# 				[0, 0, 0, 1]])

				# Translation vector from the marker 0 to the nearest circle (0 to point)
				t0p = np.array([x0p, y0p, z0p])
				t0pHomogeneous = np.array([x0p, y0p, z0p, 1])

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
				tcpProjected, _ = cv2.projectPoints(t0p, poseId0[0], poseId0[1], cameraMatrix, distCoeffs)

				# Reshape the translation vector to ease its manipulation
				tcpProjected = np.array(tcpProjected).flatten()
				# print(tcpProjected)

				# Draw the point in the frame
				cv2.circle(frame,(int(tcpProjected[0]),int(tcpProjected[1])),5,(0,0,255),thickness=-1)

				
				# The purpose now is to find the location every 3D points with respect to the camera frame and then project them to the image plane (2D)
				# Each point is separated by 3.6cm, so I'll use t0p as an initial reference

				# Matrix to store all the translation vectors from the camera to the selected point
				tcpointsMatrix = np.zeros((dotsHeight,dotsWidth,2), dtype=np.float32)
				# The 2 in np.zeros is to store the points coordinates (x,y)

				# # Store the first point in the matrix (camera to each points)
				# tcpointsMatrix[0][0] = tcpProjected
				
				# Auxiliar variables
				xAux = 0
				yAux = 0

				# Let's iterate over the 6x6 matrix
				# The location of the translation vectors from the marker 0 to each circle on the whiteboard is calculated using the first t0p as reference
				for row in range(dotsHeight):

					for column in range(dotsWidth):

						# According to the marker 0’s reference frame, the points that share a row will have the same value in x and different values in y (considering the 3.6cm separation between them)
						# If you don't get it, check the image where the coordinates of marker 0 are drawn

						currentPoint = np.array([x0p + xAux, y0p + yAux, z0p])

						currentTcpProjected, _ = cv2.projectPoints(currentPoint, poseId0[0], poseId0[1], cameraMatrix, distCoeffs)
						currentTcpProjected = np.array(currentTcpProjected).flatten()
						tcpointsMatrix[row][column] = currentTcpProjected

						# Draw the point in a clean frame (originalFrame2)
						cv2.circle(originalFrame2,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)
						cv2.circle(frame,(int(currentTcpProjected[0]),int(currentTcpProjected[1])),5,(0,0,160),thickness=-1)

						# Actualize the column value
						yAux += distanceBetweenDots

					# Actualize the row value
					xAux += distanceBetweenDots

					yAux = 0
				xAux = 0

				# Note that I've calculated the first tcpProjected again

				# Apply the transformation matrix in the frame
				transformation2 = cv2.warpPerspective(originalFrame2,transformationMatrix,(frameWidth,frameHeight))
				
				# The problem now is that since it is constantly detecting the ArUco markers (which can be very unstable depending on the ambient light), 
				# the matrix of circles on the board appears to move… so if I want to identify a line there, there could be false detections.
				# Therefore, what I could implement is to continue detecting the points on the board, but the matrix I will use to detect the lines will be an average of each of the points, 
				# taking x matrices of points.
				# This way, I think that the detection of the lines will be more stable

				if countSetOfTcpMatrix < maxSetOfTcpMatrix:
					setOfTcpointsMatrix.append(tcpointsMatrix)
					countSetOfTcpMatrix += 1
				elif countSetOfTcpMatrix == maxSetOfTcpMatrix:

					# Create a new frame variable to maintain the frame with the circles. This is necessary because when the while loop restarts, originalFrame3 resets
					averageMatrixFrame = originalFrame3.copy()
					
					# Perform an element-wise operation across every matrix in the set
					for row in range(dotsHeight):
						for column in range(dotsWidth):
							auxList = []
							for matrix in setOfTcpointsMatrix:
								auxList.append(matrix[row][column])
							auxMatrix = np.array(auxList)
							averageTcpMatrix[row][column] = np.mean(auxMatrix, axis = 0)

							# Draw the new matrix on the frame
							cv2.circle(averageMatrixFrame,(int(averageTcpMatrix[row][column][0]),int(averageTcpMatrix[row][column][1])),5,(255,0,0),thickness=-1)
				
					# Restart the counter and store the new matrices again
					countSetOfTcpMatrix = 0
					setOfTcpointsMatrix = []
					
					# cv2.warpPerspective cannot be applied if the averageTcpMatrix is not populated
					# This only affects the beginning of the process because the averageTcpMatrix starts as empty
					# Therefore, this boolean ensures that we don't display the first transformed average frame until the values in averageTcpMatrix are set
					firstAverageMatrixFound = True
				
					# Apply the transformation matrix in the frame
					transformation3 = cv2.warpPerspective(averageMatrixFrame,transformationMatrix,(frameWidth,frameHeight))

					# Adapt the perspective of the points in averageTcpMatrix. This way we can use these points in the cropped frame where the whiteboard is isolated from the rest of the image
					averageTcpMatrixTransformed = cv2.perspectiveTransform(averageTcpMatrix, transformationMatrix)


					# The format of the detected lines has to be compatible with the dot and boxes game engine used

					# When key 'p' is pressed nextStep == True (Note that I'm inside countSetOfTcpMatrix == maxSetOfTcpMatrix)
					if nextStep == True:
						
						# Now, I'll check each segment between the points to check if there is a line drawn

						# With this boolean variable, I'll be able to determine if a line was detected (they have to update every time I check the whiteboard)
						verticalLineDetected = False
						horizontalLineDetected = False

						# With this boolean variable, I'll be able to determine if a NEW line was detected (they have to update every time I check the whiteboard)
						newVerticalLineDetected = False
						newHorizontalLineDetected = False

						# Check horizontal lines
						for row in range(dotsHeight):

							# Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [x, y]
							lastDot = [None, None]

							# Define line's type (It could be vertical or horizontal)
							lineType = 'horizontal'

							for column in range(dotsWidth):
								# If it's the first row, do nothing
								if lastDot == [None, None]:
									lastDot = [averageTcpMatrixTransformed[row][column][0], averageTcpMatrixTransformed[row][column][1]]
								else:
									# Crop the image to get the particular segment between the two points
									currentRectangle, topLeftRect, bottomRightRect= crop_image(lastDot, averageTcpMatrixTransformed[row][column], lineType)

									# Apply Otsu's automatic thresholding
									threshInv = apply_thresholding(currentRectangle)

									# Detect if there is a line in the selected segment
									lineDetection = detect_lines(threshInv)

									# Draw the rectangle
									draw_rectangle(topLeftRect, bottomRightRect)

									# Draw each detected line
									if lineDetection is not None:
										for i in range(0, len(lineDetection)):
											l = lineDetection[i][0]
											cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)

									# cv2.imshow('Horizontal lines detection', threshInv)
									# cv2.waitKey(0)
									# cv2.destroyAllWindows()

									# Update the past point
									lastDot = [averageTcpMatrixTransformed[row][column][0], averageTcpMatrixTransformed[row][column][1]]

									# Get the dnbpy contention of the detected line
									if lineDetection is not None:
										horizontalLineDetected = True # A line was detected
										detectedHorizontalLineDnbpyConv = int(horizontalLinesMatrix[row][column-1])
										# print(f'{detectedHorizontalLineDnbpyConv} - {lineType}')
										# [column-1] because, in the first iteration of this loop, lastDot = [None, None]

										# Update the line detection status if necessary
										if detectedHorizontalLineDnbpyConv not in linesDetectionStatus:
											linesDetectionStatus.append(detectedHorizontalLineDnbpyConv)
											newHorizontalLineDetected = True # A NEW line was detected

							
						# Check vertical lines
						for column in range(dotsWidth):

							# Auxiliary variable 'lastDot': If it's the first dot in the row, 'lastDot' is set to None. Otherwise, 'lastDot' is set to the coordinates [x, y]
							lastDot = [None, None]

							# Define line's type (It could be vertical or horizontal)
							lineType = 'vertical'

							for row in range(dotsHeight):
								# If it's the first row, do nothing
								if lastDot == [None, None]:
									lastDot = [averageTcpMatrixTransformed[row][column][0], averageTcpMatrixTransformed[row][column][1]]
								else:
									# Crop the image to get the particular segment between the two points
									currentRectangle, topLeftRect, bottomRightRect = crop_image(lastDot, averageTcpMatrixTransformed[row][column], lineType)
									
									# Apply Otsu's automatic thresholding
									threshInv = apply_thresholding(currentRectangle)

									# Detect if there is a line in the selected segment
									lineDetection = detect_lines(threshInv)

									# Draw the rectangle
									draw_rectangle(topLeftRect, bottomRightRect)

									# Draw each detected line
									if lineDetection is not None:
										for i in range(0, len(lineDetection)):
											l = lineDetection[i][0]
											cv2.line(currentRectangle, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)

									# cv2.imshow('Vertical lines detection', threshInv)
									# cv2.waitKey(0)
									# cv2.destroyAllWindows()

									# Update the past point
									lastDot = [averageTcpMatrixTransformed[row][column][0], averageTcpMatrixTransformed[row][column][1]]
									
									# Get the dnbpy contention of the detected line
									if lineDetection is not None:
										verticalLineDetected = True # A line was detected
										detectedVerticalLineDnbpyConv = int(verticalLinesMatrix[row-1][column])
										# print(f'{detectedVerticalLineDnbpyConv} - {lineType}')
										# [row-1] because, in the first iteration of this loop, lastDot = [None, None]
										
										# Update the line detection status if necessary
										if detectedVerticalLineDnbpyConv not in linesDetectionStatus:
											linesDetectionStatus.append(detectedVerticalLineDnbpyConv)
											newVerticalLineDetected = True # A NEW line was detected

						# print(linesDetectionStatus)

						# If I want to detect the lines again, I should press 'p' once more
						# nextStep = False

						# The board status was checked for the first time
						firstCheck = True

						# Save the modified version of transformation1 (which is always updating), so we can show it below
						# It's an image of the detection in the moment I press the 'p' key
						transformation1Modified = transformation1.copy()
				
					# Press the 'm' key to begin the game
					# (Note that I'm still inside countSetOfTcpMatrix == maxSetpOfTcpMatrix)
					if startGame == True and nextStep == True:
						if not gameBegan:
								# If the board is empty...
								if verticalLineDetected == False and horizontalLineDetected == False:
									print('La pizarra se encuentra vacía. Por lo tanto, la partida ha comenzado.')
									print('Importante: la detección aún no se realiza de forma automática, por lo que se requiere presionar la letra "p" para efectuar el chequeo.')
									print('----------------------------------------------------------------------------------')
									gameBegan = True # The board is empty --> The game can begin
									# If a line is drawn on the whiteboard, linesDetectionStatus will not be empty, even if we later erase the line from the whiteboard. Therefore, in order to play, this list must be empty.
									linesDetectionStatus = []
								else:
									print('No es posible comenzar el juego porque hay líneas en la pizarra. Borre las líneas y presione "p" para realizar el chequeo.')
						# If the whiteboard has been checked and is empty, execute the following if statement
						if gameBegan:
								# Get the current player
								currentPlayer = game.get_current_player()
								
								# Execute once per turn
								if oneTime == True:
									print(f'\nTurno del {currentPlayer.lower()}.')
									oneTime = False

								# If the player makes the move...
								# If a new line that wasn't part of linesDetectionStatus appears, then execute the following code
								# That new line was stored in the last index of linesDetectionStatus
								if currentPlayer == players[0]:
									if newHorizontalLineDetected == True or newVerticalLineDetected == True:
										oneTime = True

										# A new line was detected, so it has to be the last one in the list
										playerMove = linesDetectionStatus[-1]

										# Here is an issue to consider: If I drew three lines and then pressed ‘p’, only the last one will be taken into account. 
										# But of course, this breaks the entire logic because the list would have two other lines that are supposedly already drawn, but in reality, they are not.
										# So it would be: I draw a line, press ‘p’, draw a line, press ‘p’, and so on.

										# Make the move in the dnb engine
										game.select_edge(playerMove, currentPlayer)

										print(f'Movimiento del {players[0]}: {playerMove}.')
								
								# If the robot makes the move...
								elif currentPlayer == players[1]:
									
									if robotMoveGenerated == False:
										# Get the move for the robot
										robotMove = minimaxAgent.select_edge(game.get_board_state(), game.get_score(currentPlayer), game.get_score(players[0]))
										robotMoveGenerated = True # Avoid re-entering this if statement in the same turn
										print(f'{robotMove} (Auxiliar)')

										# I need to know whether it's a vertical or horizontal line
										# Based on the robot's movement, I have to find between which two points the line should be drawn
										pointsIndexforLine, lineType2 = dnb_conv_to_points_index(robotMove) # Indices of both points, which are related to the 6x6 distance matrix
										
										# Use those indices in the 6x6 distance matrix from the base frame of the robot to the circles on the whiteboard 
										# to get the initial and final points of the line.
										initialPoint = tbpointsMatrix[int(pointsIndexforLine[0][0][0])][int(pointsIndexforLine[0][0][1])]
										finalPoint = tbpointsMatrix[int(pointsIndexforLine[0][1][0])][int(pointsIndexforLine[0][1][1])]

										print(f'Punto inicial: {initialPoint}')
										print(f'Punto final: {finalPoint}')
										print(lineType2)

										# Interpolate the line between the initial and final point
										interpolationPointsArray = simple_interpolator(initialPoint, finalPoint, lineType2) # Returns a numpy array with a shape of (numInterpolatedPoints, 3)

										# Send the instructions to the motors via serial communication with the ESP32
										thread_serial_communication(interpolationPointsArray)
										
									
									# If the robot's move has already been generated...
									else:
										# If a new line was detected...
										if newHorizontalLineDetected == True or newVerticalLineDetected == True:
											# If the new detected line matches the robot move value in dnb engine convention...
											if linesDetectionStatus[-1] == robotMove:

												# Make the move in the dnb engine
												game.select_edge(robotMove, currentPlayer)

												print(f'Movimiento del {players[1]}: {robotMove}.')

												robotMoveGenerated = False # The turn is completed, so robotMoveGenerated restarts

												oneTime = True
											else:
												# If the new detected line doesn't match robotMove, delete it from the list and inform the user
												linesDetectionStatus.pop()
												print('No se dibujó la línea en el casillero correcto. Bórrela de la pizarra.')
					else:
						# If 'm' wasn't pressed, but 'p' was, then...
						if nextStep == True and startGame == False and gameBegan == False:
							print('Se chequeó la pizarra, pero la partida no comenzó. Presione la letra "m".')
						# If 'm' was pressed, but 'p' wasn't, then...
						elif startGame == True  and gameBegan == False and alreadyDenied == False:
							print('Presione la letra "p" para efectuar el chequeo.')
							alreadyDenied = True # Avoid entering this loop again

					# If I want to detect the lines again, I should press 'p' once more
					nextStep = False

	###### Considerations
	# Be careful with padding… If I go beyond the image limit, it returns an empty array [].
	# In this logic, only one line can be detected at a time. Considering that I have to press ‘p’ each time I want to make the detection.
	# When I apply thresholding, I am doing it on the transformation1 image that has the lines delimiting the identification zones, which shouldn’t be the case.
	# Thresholding should be done on the clean image (you can see the lines of these zones in the thresholding of the vertical lines).
	# When it’s the robot’s turn: I generate the movement, send it to the robot, check if the robot has already drawn it or not (this is currently manual, by pressing ‘p’), if it has and it matches the generated movement, I set it as a movement in the game engine (game.select_edge) and show which movement was made.
	# If the program stops with the serial port communication between this script and the ESP32 in the middle of its development, you need to disconnect and reconnect the ESP32 cable to restart the communication, otherwise the servomotors will continue with the last values sent to them.

	###### To Do
	# If a player completes a box, the score should be displayed.
	# The code should be separated into different classes that run in a main.py file or something similar.
	# Correct the error in the distances from the arm to the board. I imagine this is due to the error in the motor movements, servo backlash, and error in the length of the links.


	else:
		print('No se pudo leer un frame...')

	# Display the resulting frame
	cv2.imshow('frame',frame)

	if squareFound == True:
		# Display the transformed frame
		cv2.imshow('Juego',transformation1)
		cv2.imshow('Juego2',transformation2)

		if firstAverageMatrixFound == True:
			cv2.imshow('Juego3',transformation3)
		
		if firstCheck == True:
			# Display the detected lines (note that all rectangles were extracted from transformation1 without using the copy function)
			cv2.imshow('Detected lines',transformation1Modified)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

else:
	print('----------------------------------------------------------------------------------')
	print('La partida terminó.\n')

	scorePlayer = game.get_score(player)
	scoreRobot = game.get_score(robot)

	print('Resultados:')
	print(f'{player}: {scorePlayer} puntos ---- {robot}: {scoreRobot}.\n')

	if scorePlayer > scoreRobot:
		print(f'¡Felicitaciones {player.lower()}, eres el ganador!')
	elif scoreRobot > scorePlayer:
		print(f'Ganó el robot... La humanidad está perdida...')
	elif scoreRobot == scorePlayer:
		print('¡Es un empate!')

# Close down the video stream
cap.release()
cv2.destroyAllWindows()