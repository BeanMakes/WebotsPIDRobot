import matplotlib.pyplot as plt
import RRT_Algorithm

if __name__ == "__main__":
    start = [0, 0]
    end = [8,8]

    rrt = RRT_Algorithm.RTT([])

    finalNode = rrt.findPath(start,end)

    pathX = []
    pathY = []
    while finalNode:
        temp = [finalNode.x,finalNode.y]
        pathX.insert(0,finalNode.x)
        pathY.insert(0,finalNode.y)
        finalNode = finalNode.previous

    print([pathX,pathY])
    plt.plot(pathX,pathY)
    plt.show()