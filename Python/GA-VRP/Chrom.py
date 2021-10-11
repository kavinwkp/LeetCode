from math import fabs
from VRP import VRP
import random

class Chrom:
    def __init__(self, vrp) -> None:
        self.gene = []
        self.nodeList = vrp.nodeList.copy()
        self.uavList = vrp.uavList.copy()
        self.distance = vrp.dis.copy()
        self.nodeNum = len(self.nodeList)
        self.uavNum = len(self.uavList)
        self.mileage = [0] * self.uavNum
        self.load = [0] * self.uavNum
        self.valid = True
        self.k1 = vrp.k1
        self.k2 = vrp.k2
        self.k3 = vrp.k3
        self.time = 0
        self.length = 0
        self.cnt = 0

        for i in range(self.nodeNum - 1):   # 第一个点是配送中心
            self.gene.append(i + 1)

        # random.shuffle(self.gene)

        # index = 0
        # for i in range(self.uavNum - 1):
        #     demandSum = 0
        #     while True:
        #         if index == len(self.gene):
        #             break
        #         demandSum = demandSum + self.nodeList[self.gene[index]].demand
        #         if demandSum > self.uavList[i].capacity:
        #             break
        #         index += 1
        #     self.gene.insert(index, 0)
        self.gene.insert(2, 0)
        self.update()

    def update(self):
        for i in range(self.uavNum):
            self.mileage[i] = self.load[i] = 0
        uavIdx = 0
        last = 0
        for i in range(len(self.gene)):
            if self.gene[i] == 0:
                self.mileage[uavIdx] += self.distance[last][0]
                uavIdx += 1
                last = 0
            else:
                self.mileage[uavIdx] += self.distance[last][self.gene[i]]
                self.load[uavIdx] += self.nodeList[self.gene[i]].demand
                last = self.gene[i]
                if self.load[uavIdx] > self.uavList[uavIdx].capacity:
                    self.valid = False
                    return
        self.mileage[uavIdx] += self.distance[last][0]
        
        self.time = 0
        self.length = 0
        self.cnt = 0
        for mile in self.mileage:
            self.time = max(self.time, mile)
            self.length += mile
            if fabs(mile - 0.0) > 1e-6:
                self.cnt += 1
        self.valid = True

    def mutation(self):
        i = 0
        j = -1
        self.gene[0], self.gene[-1] = self.gene[-1], self.gene[0]
        t_time = self.time
        t_length = self.length
        t_cnt = self.cnt
        if self.valid == False:
            self.gene[0], self.gene[-1] = self.gene[-1], self.gene[0]
            self.time = t_time
            self.length = t_length
            self.cnt = t_cnt

        

    def fitness(self):
        if self.valid == False:
            return 1e8
        else:
            return self.k1 * self.time + self.k2 * self.length + self.k3 * self.cnt

    def infoNode(self):
        for node in self.nodeList:
            node.info()
    
    def infoGene(self):
        print(self.gene)
    

def main():
    vrp = VRP()
    vrp.addNode(15.58, 7.1, 0.9)
    vrp.addNode(10.26, 6.08, 1.7)
    vrp.addNode(17.54, 14.12, 0.4)
    vrp.addNode(16.92, 5.3, 2.7)
    vrp.addNode(15, 13.3, 2.8)
    # vrp.infoNode()
    vrp.addUAV(10, 20)
    vrp.addUAV(10, 20)
    vrp.solve()

    chrom = Chrom(vrp)
    chrom.infoNode()
    chrom.infoGene()

    chrom.mutation()
    chrom.infoGene()



if __name__ == '__main__':
    main()