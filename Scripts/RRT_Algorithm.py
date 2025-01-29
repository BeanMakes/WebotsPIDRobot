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
        return random.uniform(minX, maxX+1), random.uniform(minY, maxY+1)

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
    
    def isThereDirectPath(self,node, endNode)->bool:
        for obs in self.obsticles:
            if self.doIntersect(node, endNode, obs[0], obs[1]):
                return False
        return True
    
    # Given three collinear points p, q, r, the function checks if  
    # point q lies on line segment 'pr'  
    def onSegment(self,p, q, r): 
        if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
               (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
            return True
        return False
      
    def orientation(self,p, q, r): 
        # to find the orientation of an ordered triplet (p,q,r) 
        # function returns the following values: 
        # 0 : Collinear points 
        # 1 : Clockwise points 
        # 2 : Counterclockwise 
          
        # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/  
        # for details of below formula.  
          
        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
        if (val > 0): 
              
            # Clockwise orientation 
            return 1
        elif (val < 0): 
              
            # Counterclockwise orientation 
            return 2
        else: 
              
            # Collinear orientation 
            return 0
      
    # The main function that returns true if  
    # the line segment 'p1q1' and 'p2q2' intersect. 
    def doIntersect(self,p1,q1,p2,q2): 
          
        # Find the 4 orientations required for  
        # the general and special cases 
        o1 = self.orientation(p1, q1, p2) 
        o2 = self.orientation(p1, q1, q2) 
        o3 = self.orientation(p2, q2, p1) 
        o4 = self.orientation(p2, q2, q1) 
      
        # General case 
        if ((o1 != o2) and (o3 != o4)): 
            return True
      
        # Special Cases 
      
        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1 
        if ((o1 == 0) and onSegment(p1, p2, q1)): 
            return True
      
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1 
        if ((o2 == 0) and onSegment(p1, q2, q1)): 
            return True
      
        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2 
        if ((o3 == 0) and onSegment(p2, p1, q2)): 
            return True
      
        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2 
        if ((o4 == 0) and onSegment(p2, q1, q2)): 
            return True
      
        # If none of the cases 
        return False
    
    def findPath(self,start, goal):

        nodeList = [Node(start[0],start[1])]
        endGoal = Node(goal[0],goal[1])
        counter = 1
        done = False

        limit = 1
        while not done:
            # print(counter)
            point = self._generatePoint(5,-5,5,-5)
            currentNode = Node(point[0],point[1])
            nearestNode = self.nearestNode(currentNode.x, currentNode.y,nodeList)
            if self.isThereDirectPath(nearestNode, currentNode):
                currentNode.previous = nearestNode
                nodeList.append(currentNode)
                if currentNode.x >= endGoal.x-0.15 and currentNode.x <= endGoal.x+0.15 and currentNode.y >= endGoal.y-0.15 and currentNode.y <= endGoal.y+0.15:
                    done = True
            counter+=1
        return currentNode