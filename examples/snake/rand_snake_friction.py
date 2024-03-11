# This file is used to generate random snake friction configurations and support their reproducing features.

from typing import Union
from math import sin, cos
import copy, random
import numpy as np

"""
TO DO: Support different magnitude values
"""
class FrictionalSnake:
    def __init__(
            self,
            linkNum: int,
            magnitude: int, # Uniform magnitude for all links
            frictionRate: float, # How possible for a link to be frictional. Should be high.
            flagMutateRate: float, # How possible for a link to change its frictional status. Should be low.
            dirMutateRate: float # How possible for a link friction to change its direction. Should be low.
    ):
        self.linkNum = linkNum
        self.magnitude = [magnitude] * linkNum
        self.frictionRate = frictionRate
        self.flagMutateRate = flagMutateRate
        self.dirMutateRate = dirMutateRate

        self.direction = self.generateRandDir()
        self.flag = self.generateRandFlag()
        self.frictionConcigurations = self.snakeFriction()

        self.velocity = 0 # Wait to be changed after each run

    def generateRandDir(self):
        dir = {}
        for i in range(self.linkNum):
            dir[i] = np.random.randint(0, 90)
        return dir
    
    def generateRandFlag(self):
        flag = {}
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.frictionRate:
                flag[i] = 1
            else:
                flag[i] = 0
        return flag
    
    def snakeFriction(self) -> dict:
        print(self.flag)
        print(self.direction)
        magnitude_flag = [x * y for x, y in zip(self.magnitude, self.flag)]
        frictionConfigurations = {}
        # i and j represents for the snake and link, seperately.
        for i in range(self.linkNum):
            frictionConfigurations[i] = {
                "lateralFriction": 1,
                "anisotropicFriction": [magnitude_flag[i]*sin(self.direction[i]), magnitude_flag[i]*cos(self.direction[i]), 0.01],
                "angularDamping": 3,
                'restitution': 3.0
            }
        return frictionConfigurations
    
    def mutateFlag(self):
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.flagMutateRate:
                self.flag[i] = 1 - self.flag[i]

    def mutateDir(self):
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.dirMutateRate:
                self.direction[i] = np.random.randint(0, 90)

    def setVelocity(self, velocity):
        self.velocity = velocity

    def crossover(self, other):
        assert isinstance(other, FrictionalSnake), f"other has to be of type FrictionalSnake"

        startPoint = np.random.randint(0, self.linkNum)
        endPoint = np.random.randint(0, self.linkNum) % (self.linkNum - startPoint) + startPoint

        newSnake1 = copy.deepcopy(self)
        newSnake2 = copy.deepcopy(other)
        tmpDir = newSnake1.direction[startPoint:endPoint + 1]
        tmpFlag = newSnake1.flag[startPoint:endPoint + 1]
        newSnake1.direction[startPoint:endPoint + 1] = newSnake2.direction[startPoint:endPoint + 1]
        newSnake2.direction[startPoint:endPoint + 1] = tmpDir
        newSnake1.flag[startPoint:endPoint + 1] = newSnake2.flag[startPoint:endPoint + 1]
        newSnake2.flag[startPoint:endPoint + 1] = tmpFlag

        newSnake1.frictionConcigurations = newSnake1.snakeFriction()
        newSnake2.frictionConcigurations = newSnake2.snakeFriction()

        return [newSnake1, newSnake2]
     

class FrictionalSnakeGroup:
    def __init__(
        self,
        frictionalSnakes: list,
    ):
        self.snakeNum = len(frictionalSnakes)
        self.frictionalSnakes = frictionalSnakes
        self.isFiltered = True

    def __add__(self, other):
        assert isinstance(other, FrictionalSnakeGroup), f"other has to be of type FrictionalSnakeGroup"
        newGroup = FrictionalSnakeGroup(self.frictionalSnakes + other.frictionalSnakes)
        newGroup.isFiltered = False
        return newGroup

    def linkCrossover(self):
        # exchange segments between two snakes, return crossovered snakes.
        # REMEMBER to merge them into one group before execute the simulation.
        crossoveredSnakes = []

        snakeNumList = list(range(self.snakeNum))
        random.shuffle(snakeNumList)
        snakePairs = []
        for i in range(0, len(snakeNumList), 2):
            if i+1 < len(snakePairs):
                snakePairs.append([snakeNumList[i], snakeNumList[i+1]])

        for pair in snakePairs:
            snake1, snake2 = self.frictionalSnakes[pair[0]], self.frictionalSnakes[pair[1]]
            crossoveredSnakes.append(snake1.crossover(snake2))

        return FrictionalSnakeGroup(crossoveredSnakes)

    def mutate(self):
        for snake in self.frictionalSnakes:
            snake.mutateFlag()
            snake.mutateDir()

    def filterSlowSnakes(self):
        # filter out snakes with velocity slower than threshold
        if not self.isFiltered:
            velocities = [snake.velocity for snake in self.frictionalSnakes]
            probabilities = list(map(lambda x: x*x, velocities))
            sumOfProbabilities = sum(probabilities)
            probabilities = [x / sumOfProbabilities for x in probabilities]

            filteredVelocities = np.random.choice(velocities, self.snakeNum, p=probabilities)
            for snake in self.frictionalSnakes:
                if snake.velocity not in filteredVelocities:
                    self.frictionalSnakes.remove(snake)

            self.snakeNum = len(self.frictionalSnakes)
            self.isFiltered = True
        return self

    def setVelocities(self, velocities):
        for v in velocities:
            self.frictionalSnakes[v[0]].setVelocity(v[1])