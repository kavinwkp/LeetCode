package util

import "fmt"

var i = 0
var F = func(s string) int {
	fmt.Printf("本次被%s调用\n", s)
	i++
	fmt.Printf("匿名工具函数被第%v次调用\n", i)
	return i
}

func SelectByKey(text ...string) (key int) {
	for i, v := range text {
		fmt.Printf("%v %v\n", i, v)
	}
	fmt.Println("input key")
	fmt.Scanln(&key)
	return
}
