import cv2
import numpy as np

################################################################################################
############################# Functions related to the dnbpy library #############################
################################################################################################

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

# Using the dnb convention, determine the points for drawing each vertical and horizontal line and store them in two matrices
# Each point returned by this function corresponds to the indices of the tbpointsMatrix matrix.
# For instance, pointsOfHorizontalLines[0,0] = array([[0,0],[0,1]]) corresponds to the indices of the initial and final points of the first horizontal line in the first row
# Using these points, I can obtain the distance in cm from the base of the robot frame to those points using the tbpointsMatrix matrix
def dnb_conv_to_matrix_points(boardRows, boardColumns, dotsHeight, dotsWidth):
    # Matrices for storing the points
    pointsOfHorizontalLines = np.zeros(shape=(boardRows + 1, boardColumns,2,2)) # (6,5,2,2)
    pointsOfVerticalLines = np.zeros(shape=(boardRows, boardColumns + 1,2,2)) # (5,6,2,2)

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

# From the robot movement in dnb convention, determine between which two points the line should be drawn
def dnb_conv_to_points_index(robotMovement, horizontalLinesMatrix, verticalLinesMatrix, pointsHorizontalLinesMatrix, pointsVerticalLinesMatrix):
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

##############################################################################################################
########################################## Line detection functions ##########################################
##############################################################################################################

def crop_image(point1, point2, lineType, frameCopyToCut, resolution, boardNumRows, boardNumCols):

    # frameCopyToCut = frame.copy()
    # We don't want to make a copy to keep rectangles related to frameCopyToCut. A copy would make the cut independent of the whole image.
    
    # print(frameCopyToCut.shape) # (2080, 2920, 3)

    # Set the frame values according to the resolution
    if resolution == '1080p':
        frameHeight = 1080
        frameWidth = 1920
    elif resolution == '480p':
        frameHeight = 480
        frameWidth = 640

    # The padding formula is the same for both line types

    if lineType == 'horizontal':

        # Define corners for the new rectangle (considering a padding in the y axis)
        padding = (frameHeight/boardNumRows)*0.2
        # Be careful with the padding, if it surpass the borders of the image, an error will raise in gray

        # Select two opposite vertices of the rectangle
        topLeftRectangle = [point1[0], point1[1] - padding]
        bottomRightRectangle = [point2[0], point2[1] + padding]

        # https://stackoverflow.com/questions/9084609/how-to-copy-a-image-region-using-opencv-in-python
        # https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python

    elif lineType == 'vertical':
        # Define corners for the new rectangle (considering a padding in the x axis)
        padding = (frameHeight/boardNumRows)*0.2
        # Be careful with the padding, if it surpass the borders of the image, an error will raise in gray

        # Select two opposite vertices of the rectangle
        topLeftRectangle = [point1[0] - padding, point1[1]]
        bottomRightRectangle = [point2[0] + padding, point2[1]]

        # https://stackoverflow.com/questions/9084609/how-to-copy-a-image-region-using-opencv-in-python
        # https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python

    # Crop the image using numpy slicing
    # currentRectangle = transformation1[int(topLeftRectangle[1]):int(bottomRightRectangle[1]), int(topLeftRectangle[0]):int(bottomRightRectangle[0])].copy()
    currentRectangle = frameCopyToCut[int(topLeftRectangle[1]):int(bottomRightRectangle[1]), int(topLeftRectangle[0]):int(bottomRightRectangle[0])]
    # Numpy uses (row, col) notation instead. Hence ---> transformation1[row,col] = transformation[y,x]

    # Return the cropped image
    return currentRectangle, topLeftRectangle, bottomRightRectangle

def apply_thresholding(cropedImage):
    # Convert to grayscale
    gray = cv2.cvtColor(cropedImage, cv2.COLOR_BGR2GRAY)

    # Apply Otsu's automatic thresholding
    (T, threshInv) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    # Return the thresholded image
    return threshInv

def detect_lines(thresholdedImage):

    # Get the larger dimension of the frame
    # The greater axis corresponds to the direction of the line
    maxDimensionValue = max(thresholdedImage.shape)

    # https://stackoverflow.com/questions/41329665/linesegmentdetector-in-opencv-3-with-python
    # lineDetector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_ADV)
    # lines = lineDetector.detect(gray)[0]
    # draw = lineDetector.drawSegments(gray, lines)

    # lines = cv2.HoughLines(threshInv, 1, np.pi/180, 80, None, 0, 0)

    # if lines is not None:
    #   for i in range(0, len(lines)):
    #       rho = lines[i][0][0]
    #       theta = lines[i][0][1]
    #       a = np.cos(theta)
    #       b = np.sin(theta)
    #       x0 = a * rho
    #       y0 = b * rho
    #       pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    #       pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    #       cv2.line(currentRectangle, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

    # https://docs.opencv.org/4.10.0/dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a
    # https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html

    # Apply the Probabilistic Line Transform to detect if a line was drawn
    # linesP = cv2.HoughLinesP(thresholdedImage, 1, np.pi / 180, 40, None, 50, 10)

    rho = 1              # Distance resolution of the accumulator in pixels
    theta = np.pi/180    # Angle resolution of the accumulator in radians
    threshold = 100       # Only lines that are greater than threshold will be returned
    minLineLength = maxDimensionValue*0.5   # Line segments shorter than that are rejected
    maxLineGap = 5     # Maximum allowed gap between points on the same line to link them

    linesP = cv2.HoughLinesP(thresholdedImage, rho = rho, theta = theta, threshold = threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)

    # It is only configured for 1080p


    # linesP = Output vector of lines. Each line is represented by a 4-element vector (x1,y1,x2,y2),
    # where (x1,y1) and (x2,y2) are the ending points of each detected line segment

    return linesP

def draw_rectangle(topLeftRectangle, bottomRightRectangle, frame):
    # Draw the rectangle in the image
    cv2.rectangle(frame, (int(topLeftRectangle[0]), int(topLeftRectangle[1])), (int(bottomRightRectangle[0]), int(bottomRightRectangle[1])), (30, 0, 0), 1)
    # Drawing the rectangle in the frame doesn't have to be done before detecting the lines

#################################################################################################################################
################### Matrix for storing all the distances from the robotic arm to the points on the whiteboard ###################
#################################################################################################################################

def get_distance_matrix_from_robot_to_points(xbp1, ybp1, zbp, dotsHeight, dotsWidth, distanceBetweenDots):
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
    
    return tbpointsMatrix