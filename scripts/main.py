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

# Create a new game instance
game = dnb_game.dnbGame(boardSize = boardSize, playerName = playerName, 
						difficulty = difficulty, distanceBetweenDots = distanceBetweenDots, 
						markerSizeInCM = markerSizeInCM, firstPlayer = firstPlayer)

# Continue the loop as long as the game isn't over
while not game.has_finished():

	# Obtain frame
	frame = cap.get_frame()

	if frame is not None:
    
		frame2, overlay = game.detect_board(frame)


		cv2.imshow('frame',frame2)
		cv2.imshow('overlay', overlay)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break



cap.release_camera()