#!/usr/bin/env python3

class Person:
    def __init__(self, name, age):
        self.name = name
        self.age = age

    def sayhello(self):
        print('My name is:', self.name)
        print('My age is:', self.age)

def main():
    p = Person('kavin', 34)
    p.sayhello()

if __name__ == '__main__':
    main()