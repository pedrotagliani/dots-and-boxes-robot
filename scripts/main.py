# Import libraries
from modules import camera, dnb_game, aruco
import cv2

# Capture webcam
videoSource = 0
cap = camera.myCamera(videoSource)

# Board's configuration
dotsWidth = 6
dotsHeight = 6
distanceBetweenDots = 3.6 # Distance in centimetre
markerSizeInCM = 3.8 # Size of the marker in centimetre

# Game variables
boardSize = (dotsWidth, dotsHeight)
playerName = 'Gallardo'
difficulty = 'hard'
firstPlayer = 0 # Player begins

# Obtain width and height of the video capture
frameWidth = cap.width
frameHeight = cap.height

# Create a new game instance
game = dnb_game.dnbGame(boardSize = boardSize, playerName = playerName, 
						difficulty = difficulty, distanceBetweenDots = distanceBetweenDots, 
						markerSizeInCM = markerSizeInCM, firstPlayer = firstPlayer)

# Continue the loop as long as the game isn't over
while not game.has_finished():

	# Obtain frame
	frame = cap.get_frame()

	if frame is not None:
    
		framewithMarkers, frameOverlay, transformation1 = game.detect_board(frame)

		if framewithMarkers is not None:
			cv2.imshow('Frame with markers', framewithMarkers)

		if frameOverlay is not None:
			cv2.imshow('Original frame with overlay', frameOverlay)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break



cap.release_camera()