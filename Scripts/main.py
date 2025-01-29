import matplotlib.pyplot as plt
import RRT_Algorithm

if __name__ == "__main__":
    start = [0, 0]
    end = [1,0]
    box = [[RRT_Algorithm.Node(0.45,-1),RRT_Algorithm.Node(0.75,-1)],
        [RRT_Algorithm.Node(0.45,1),RRT_Algorithm.Node(0.75,1)],
        [RRT_Algorithm.Node(0.45,-1),RRT_Algorithm.Node(0.45,1)],
        [RRT_Algorithm.Node(0.75,-1),RRT_Algorithm.Node(0.75,1)]]
    obsticles = box
    rrt = RRT_Algorithm.RTT(obsticles)

    finalNode = rrt.findPath(start,end)

    pathX = []
    pathY = []
    while finalNode:
        temp = [finalNode.x,finalNode.y]
        pathX.insert(0,finalNode.x)
        pathY.insert(0,finalNode.y)
        finalNode = finalNode.previous

    print([pathX,pathY])
    obsticlesPlotX = [node.x for points in obsticles for node in points ]
    obsticlesPlotY = [node.y for points in obsticles for node in points]
    plt.plot(pathX,pathY)
    plt.plot(obsticlesPlotX,obsticlesPlotY)
    plt.show()