import random
import math

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.previous = None

class RTT:

    def __init__(self, obsticles):
        self.obsticles = obsticles

    def _generatePoint(self, maxX, minX, maxY, minY):
        return random.randint(minX, maxX+1), random.randint(minY, maxY+1)

    def isInObsticle(self, point) -> bool:
        for obs in self.obsticles:
            if obs[0]<= point[0] and obs[2]>= point[0] and obs[1]<=point[1] and obs[3]>=point[1]: return True
        return False
    
    def nearestNode(self,x,y,listOfCoords):
        closestDistance = 100000000
        closestNode = None
        for coord in listOfCoords:
            if closestDistance>math.sqrt((coord.x-x)**2 + (coord.y-y)**2):
                closestDistance = math.sqrt((coord.x-x)**2 + (coord.y-y)**2)
                closestNode = coord
        return closestNode

    def findPath(self,start, goal):

        nodeList = [Node(start[0],start[1])]
        endGoal = Node(goal[0],goal[1])
        counter = 1
        done = False

        limit = 1
        while not done:
            print(counter)
            point = self._generatePoint(10,-5,10,-5)
            currentNode = Node(point[0],point[1])
            nearestNode = self.nearestNode(currentNode.x, currentNode.y,nodeList)
            currentNode.previous = nearestNode
            nodeList.append(currentNode)
            if currentNode.x == endGoal.x and currentNode.y == endGoal.y:
                done = True
            counter+=1
        return currentNode