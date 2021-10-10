
class Node:
    def __init__(self, idx, x, y, demand) -> None:
        self.idx = idx
        self.x = x
        self.y = y
        self.demand = demand

    def info(self):
        print("Node [id: %d, %d, %d, %d]" % (self.idx, self.x, self.y, self.demand))


def main():
    pass

if __name__ == '__main__':
    main()