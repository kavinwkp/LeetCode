import numpy as np
from Node import Node
from UAV import UAV
from math import *


class VRP:
    def __init__(self) -> None:
        self.nodeNum = 0
        self.uavNum = 0
        self.nodeList = []
        self.nodeList.append(Node(self.nodeNum, 0, 0, 0))
        self.nodeNum = self.nodeNum + 1
        self.uavList = []
        self.dis = []
        self.k1 = 1.0
        self.k2 = 1.0
        self.k3 = 1.0

    def addNode(self, x, y, demand):
        self.nodeList.append(Node(self.nodeNum, x, y, demand))
        self.nodeNum = self.nodeNum + 1

    def infoNode(self):
        for node in self.nodeList:
            node.info()

    def addUAV(self, capacity, energyLimit):
        self.uavList.append(UAV(self.uavNum, capacity, energyLimit))
        self.uavNum = self.uavNum + 1

    def setWeight(self, k1, k2, k3):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

    def solve(self):
        self.dis = [[0] * self.nodeNum for i in range(self.nodeNum)]
        for i in range(self.nodeNum):
            for j in range(self.nodeNum):
                self.dis[i][j] = sqrt((self.nodeList[i].x - self.nodeList[j].x) * (self.nodeList[i].x - self.nodeList[j].x)
                                      + (self.nodeList[i].y - self.nodeList[j].y) * (self.nodeList[i].y - self.nodeList[j].y))


def main():
    vrp = VRP()
    vrp.addNode(10, 20, 23)
    vrp.addNode(100, 200, 23)
    vrp.infoNode()


if __name__ == '__main__':
    main()
