from Person import Person
import numpy as np
import random

# kavin = Person('kavin', 12)
# kavin.sayhello()

# test = np.zeros((2, 3), dtype=np.int)
# print(test)

# a = [1, 2, 3, 4, 5]
# print(a)
# a.insert(100, 100)
# print(a)
# a[0] += 10
# print(a)
# a[0], a[-1] = a[-1], a[0]
# print(a)

# print(max(5.5, a[0]))
import operator


class student:
    def __init__(self, name, age, weight):
        self.name = name
        self.age = age
        self.weight = weight


std1 = student("std1", 10, 55)
std2 = student("std2", 8, 30)
std3 = student("std3", 11, 65)
std4 = student("std4", 9, 80)

students = [std1, std2, std3, std4]
students.sort(key=operator.attrgetter("weight"))
for std in students:
    print(std.name)

samples = np.random.choice(['R', 'G', 'B'], size=10, p=[0.2, 0.5, 0.3])
print(samples)  