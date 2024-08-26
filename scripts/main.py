# Import libraries
from modules import camera, dnb_game, aruco
import cv2

# Capture webcam
videoSource = 0
cap = camera.myCamera('1080p',videoSource)

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
frameWidth = int(cap.width)
frameHeight = int(cap.height)

# print(frameWidth)
# print(frameHeight)

# Create a new game instance
game = dnb_game.dnbGame(boardSize = boardSize, playerName = playerName, difficulty = difficulty, 
						distanceBetweenDots = distanceBetweenDots, markerSizeInCM = markerSizeInCM, 
						frameWidth = frameWidth, frameHeight = frameHeight, firstPlayer = firstPlayer)

# Continue the loop as long as the game isn't over
while not game.has_finished():

	# Obtain frame
	frame = cap.get_frame()

	if frame is not None:

		framewithMarkers, frameOverlay, boardFrame, frameTestMarkerstoPoints, boardWithDots, boardWithDotsAverage = game.detect_board(frame)
		# le tendría que pasar la cámara, no el frame porque tengo que sacar el promedio de los círculos detectados
		
		cv2.imshow('Frame with markers', cv2.resize(framewithMarkers, (640, 480)))


		if game.boardDetected == True:
			cv2.imshow('Original frame with overlay', cv2.resize(frameOverlay, (640,480)))
			cv2.imshow('Game board', cv2.resize(boardFrame, (640,480)))
			cv2.imshow('Testing board points in each ArUco marker', cv2.resize(frameTestMarkerstoPoints, (640,480)))
			cv2.imshow('Board with dots', cv2.resize(boardWithDots, (640,480)))
			cv2.imshow('Board with dots average', cv2.resize(boardWithDotsAverage, (640,480)))

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release_camera()