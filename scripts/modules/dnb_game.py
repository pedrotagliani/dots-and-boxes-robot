import cv2
import dnbpy
from modules import utils, aruco
import pickle
import numpy as np

# Load the data from the pickle file
with open('config/camera_config.pckl', 'rb') as file:
	loadedData = pickle.load(file)

# Camera parameters obtained from calibration
cameraMatrix = loadedData[0]
distCoeffs = loadedData[1]

class dnbGame():
    # boardSize (tuple) --> (dotsWidth, dotsHeight)
    # playerName (string) --> 'Pedro', for example
    # difficulty (string) --> 'easy', 'medium' or 'hard'
    # firstPlayer (int) --> 0 (player, default value) or 1 (robot)
    # distanceBetweenDots (int) --> Distance in centimetre. For example: 3.6
    # markerSizeInCM (int) --> Size of the marker in centimetre. For example: 3.8
    # cameraMatrix and distCoeffs --> Default values obtained through calibration are specific to the Q-BOX camera
    def __init__(self, boardSize, playerName, difficulty, distanceBetweenDots, markerSizeInCM, cameraMatrix = cameraMatrix, distCoeffs = distCoeffs, firstPlayer = 0):  
        # Logical configuration
        self.playerName = playerName
        self.difficulty = difficulty
        self.firstPlayer = firstPlayer
        self.players = [self.playerName, 'Robot']

        # Board's configuration
        self.boardSize = boardSize
        self.dotsWidth = self.boardSize[0]
        self.dotsHeight = self.boardSize[1]
        self.boardNumColumns = self.dotsWidth - 1
        self.boardNumRows = self.dotsHeight - 1
        self.boardTotalLines = self.dotsHeight*self.boardNumRows + self.dotsWidth*self.boardNumColumns # If 6x6 --> 60 lines
        self.DistanceBetweenDots = distanceBetweenDots
        self.markerSizeInCM = markerSizeInCM # Size of the marker in centimetre

        # Camera parameters
        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs

        # Create a new game instances
        self.game = dnbpy.Game((self.boardNumRows, self.boardNumColumns), self.players)

        # Change the current player (default value is 0)
        self.game._current_player = self.firstPlayer

        # Select the AI according to the difficulty
        if self.difficulty == 'easy':
            self.aiAgent = dnbpy.RandomPolicy()
        elif self.difficulty == 'medium':
            self.aiAgent = dnbpy.Level2HeuristicPolicy(self.boardSize)
        elif self.difficulty == 'hard':
            self.aiAgent = dnbpy.Level3MinimaxPolicy(boardSize, depth=2, update_alpha=True)
        
    def has_finished(self):
        return self.game.is_finished() # return True or False
    
    def detect_board(self, frame):
        # Make a copy of the original frame
        frameCopy = frame.copy()

        # Overlay to draw the polygon
        overlay = frame.copy()


        # Detect ArUco markers in the video frame
        corners, ids = aruco.detect_ArUcos(frameCopy)

        # Draw ArUco markers detected
        aruco.draw_detected_ArUcos(frameCopy, corners)

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
                    cv2.drawFrameAxes(frameCopy, cameraMatrix, distCoeffs, rvec, tvec, length=8, thickness=4)

                    # I am interested in saving the pose of each marker. Later, I'll relate each one of them with every circle in the real matrix
                    markersPose.append([rvec,tvec])

                # # Overlay to draw the polygon
                # overlay = frame.copy()

                # List to store the top left marker corner sorted by ID number
                markersLeftCornersSorted = [None, None, None, None]

                for i in range(4): #0,1,2,3
                    index = ids.index(i) # if ids = [3,1,2] --> i = 3 --> index = 0
                    markersLeftCornersSorted[i] = markersLeftCorners[index]

                # Convert from list to array so I can use cv2.fillPolly
                markersLeftCornersSorted = np.array(markersLeftCornersSorted)

                # https://stackoverflow.com/questions/17241830/opencv-polylines-function-in-python-throws-exception
                cv2.fillPoly(overlay, pts = np.int32([markersLeftCornersSorted]), color=(255,215,0))

                # Blend the overlay and the original frame with cv2.addWeighted
                alpha = 0.4  # Transparency factor
                beta = 1 - alpha # Transparency factor
                gamma = 0 # Transparency factor
                frame3 = cv2.addWeighted(frame, alpha, overlay, beta, gamma)









        else:
            # Display that no ArUco markers were detected
            markersnotFoundString = 'No se han encontrado marcadores'
            cv2.putText(frameCopy,markersnotFoundString,(20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)


        return frameCopy, frame3





if __name__ == '__main__':
    pass