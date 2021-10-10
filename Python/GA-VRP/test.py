from Person import Person
import numpy as np
import random

# kavin = Person('kavin', 12)
# kavin.sayhello()

# test = np.zeros((2, 3), dtype=np.int)
# print(test)

a = [1, 2, 3, 4, 5]
# print(a)
# a.insert(100, 100)
# print(a)
# a[0] += 10
# print(a)
a[0], a[-1] = a[-1], a[0]
print(a)

print(max(5.5, a[0]))