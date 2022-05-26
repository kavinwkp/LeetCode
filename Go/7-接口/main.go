package main

import (
	"fmt"
)

type Sleeper interface {
	Sleep()
}

type Eater interface {
	Eat(foodName string)
}

type LazyAnimal interface {
	Sleeper
	Eater
}

type Dog struct {
	Name string
}

func (d Dog) Sleep() {
	fmt.Printf("Dog %s is sleeping\n", d.Name)
}

func (d Dog) Eat(foodName string) {
	fmt.Printf("Dog %s is eating %s\n", d.Name, foodName)
}

type Cat struct {
	Name string
}

func (c Cat) Sleep() {
	fmt.Printf("Cat %s is sleeping\n", c.Name)
}

func (c Cat) Eat(foodName string) {
	fmt.Printf("Cat %s is eating %s\n", c.Name, foodName)
}

func AnimalSleep(s Sleeper) {
	s.Sleep()
}

func test1() {
	var s Sleeper
	dog := Dog{Name: "小白"}
	cat := Cat{Name: "kitty"}
	s = dog
	AnimalSleep(s)

	s = cat
	AnimalSleep(s)

	sleepList := []Sleeper{
		Dog{Name: "小黑"},
		Cat{Name: "Kitty"},
	}

	for _, s := range sleepList {
		s.Sleep()
	}
	// Dog 小白 is sleeping
	// Cat kitty is sleeping
	// Dog 小黑 is sleeping
	// Cat Kitty is sleeping
}

func test2() {
	sleepList := []LazyAnimal{
		Dog{Name: "小黑"},
		Cat{Name: "Kitty"},
	}

	for _, s := range sleepList {
		s.Sleep()
		s.Eat("food")

		// 类型断言 type assert
		if dog, ok := s.(Dog); ok {
			fmt.Printf("I am a Dog, my name is %s\n", dog.Name)
		}
		if cat, ok := s.(Cat); ok {
			fmt.Printf("I am a Cat, my name is %s\n", cat.Name)
		}
	}
	// Dog 小黑 is sleeping
	// Dog 小黑 is eating food
	// I am a Dog, my name is 小黑
	// Cat Kitty is sleeping
	// Cat Kitty is eating food
	// I am a Cat, my name is Kitty
}

func test3() {
	// 空接口
	animalList := []interface{}{
		Dog{Name: "小黑"},
		Cat{Name: "Kitty"},
	}

	for _, s := range animalList {
		// 类型断言 type assert
		if dog, ok := s.(Dog); ok {
			fmt.Printf("I am a Dog, my name is %s\n", dog.Name)
		}
		if cat, ok := s.(Cat); ok {
			fmt.Printf("I am a Cat, my name is %s\n", cat.Name)
		}
	}
	// I am a Dog, my name is 小黑
	// I am a Cat, my name is Kitty
}

func MyPrint(i interface{}) {
	switch o := i.(type) {
	case int:
		fmt.Printf("%d\n", o)
	case float64:
		fmt.Printf("%f\n", o)
	case string:
		fmt.Printf("%s\n", o)
	default:
		fmt.Printf("%+v\n", o)
	}
}

func test4() {
	MyPrint(42)
	MyPrint(3.14)
	MyPrint("kavin")
	MyPrint(map[string]int{"kavin": 23})

	// 42
	// 3.140000
	// kavin
	// map[kavin:23]
}

func main() {
	// test1()
	// test2()
	// test3()
	test4()
}
