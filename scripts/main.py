# Import libraries
from modules import camera, dnb_game
import cv2

# Capture webcam
videoSource = 0
cap = camera.MyCamera('1080p',videoSource)

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
frameWidth = int(cap.frameWidth)
frameHeight = int(cap.frameHeight)

# print(frameWidth)
# print(frameHeight)

# Create a new game instance
game = dnb_game.DnbGame(boardSize = boardSize, playerName = playerName, difficulty = difficulty, 
						distanceBetweenDots = distanceBetweenDots, markerSizeInCM = markerSizeInCM, 
						camera = cap, firstPlayer = firstPlayer)

# Continue the loop as long as the game isn't over
while not game.has_finished():

	game.detect_board()

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release_camera()