# Import libraries
from modules import camera, dnb_game_v2
# from modules import camera, dnb_game

# Capture webcam
videoSource = 0
cap = camera.MyCamera('1080p',videoSource)

# Board's configuration
dotsWidth = 4
dotsHeight = 3
distanceBetweenDots = 3.6 # Distance in centimetre
markerSizeInCM = 3.8 # Size of the marker in centimetre

# Game variables
boardDotSize = (dotsWidth, dotsHeight)
playerName = 'Player'
difficulty = 'hard'
firstPlayer = 0 # Player begins

# Obtain width and height of the video capture
frameWidth = int(cap.frameWidth)
frameHeight = int(cap.frameHeight)

# print(frameWidth)
# print(frameHeight)

# Start the game
print('\n¡Hola! Ingresá tu nombre:')
playerName = input('>> ')

if len(playerName) > 0:

    print(f"\n ¡Perfecto, {playerName}!")

    # The game has started...

    # Create a new game instance
    game = dnb_game_v2.DnbGame(boardDotSize = boardDotSize, playerName = playerName, difficulty = difficulty, 
                            distanceBetweenDots = distanceBetweenDots, markerSizeInCM = markerSizeInCM, 
                            camera = cap, firstPlayer = firstPlayer)

    # Check the physical setup (manual configuration)
    game.check_setup()

    game.check_board_detection()

    # Check if the board is empty
    game.is_whiteboard_empty()

    # Just play
    game.play()

    # May be useful for debugging with dnb_game
    # Continue the loop as long as the game isn't over
    # while not game.has_finished():

    #     # Detect the board
    #     averageTcpMatrixTransformed, boardFrame = game.detect_board()

    #     # Detect lines
    #     detectedLinesList, newLineDetected = game.detect_lines(averageTcpMatrixTransformed, boardFrame)

    #     # print(detectedLinesList)
    #     # print(newLineDetected)
    #     # print('-------')
    
else:
    print(f'No ingresaste ningún nombre...')

cap.release_camera()