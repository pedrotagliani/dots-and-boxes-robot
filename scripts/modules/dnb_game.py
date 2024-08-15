import cv2
import dnbpy
import utils

class dnbGame():
    # boardSize (tuple) --> (boardNumRows, boardNumColumns)
    # playerName (string) --> 'Pedro', for example
    # difficulty (string) --> 'easy', 'medium' or 'hard'
    # firstPlayer (int) --> 0 (player) or 1 (robot)
    def __init__(self, boardSize, playerName, difficulty, firstPlayer = 0):
        self.boardSize = boardSize
        self.playerName = playerName
        self.difficulty = difficulty
        self.firstPlayer = firstPlayer
        self.players = [self.playerName, 'Robot']

        # Create a new game instances
        game = dnbpy.Game(self.boardSize, self.players)

        # Change the current player (default value is 0)
        game._current_player = self.firstPlayer

        # Select the AI according to the difficulty
        if self.difficulty == 'easy':
            aiAgent = dnbpy.RandomPolicy()
        elif self.difficulty == 'medium':
            aiAgent = dnbpy.Level2HeuristicPolicy(self.boardSize)
        elif self.difficulty == 'hard':
            aiAgent = dnbpy.Level3MinimaxPolicy(boardSize, depth=2, update_alpha=True)











if __name__ == '__main__':
    pass