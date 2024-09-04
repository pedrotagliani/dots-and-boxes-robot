# Import libraries
from modules import camera, dnb_game
import cv2
import keyboard

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

# Start the game
print('\nPresioná "s" para comenzar el juego.')
userInput = input('>> ')

if userInput == 's':

    # The game has started...

    # Create a new game instance
    game = dnb_game.DnbGame(boardSize = boardSize, playerName = playerName, difficulty = difficulty, 
                            distanceBetweenDots = distanceBetweenDots, markerSizeInCM = markerSizeInCM, 
                            camera = cap, firstPlayer = firstPlayer)

    # Check the illumination (manual configuration)
    game.check_light()

    # Check if the board is empty
    game.is_whiteboard_empty()

    print('\n------------------------------------------')
    print('Que comience el juego...')

    # Continue the loop as long as the game isn't over
    while not game.has_finished():

        # Detect the board
        averageTcpMatrixTransformed, boardFrame = game.detect_board()

        # Detect lines
        detectedLinesList, newLineDetected = game.detect_lines(averageTcpMatrixTransformed, boardFrame)

        # print(detectedLinesList)
        # print(newLineDetected)
        # print('-------')
    
else:
    print(f'Presionaste "{userInput}", pero para que el juego inicie tenés que apretar "s".')

cap.release_camera()