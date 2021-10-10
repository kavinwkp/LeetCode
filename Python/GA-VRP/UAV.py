class UAV:
    def __init__(self, idx, capacity, energyLimit) -> None:
        self.idx = idx
        self.capacity = capacity
        self.energyLimit = energyLimit
    
    def info(self):
        print("UAV [id: %d, %d, %d, %d]" % (self.idx, self.capacity, self.energyLimit))

