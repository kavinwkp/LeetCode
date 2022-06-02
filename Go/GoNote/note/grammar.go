package note

import (
	"bytes"
	"fmt"
	"sync"
	"unsafe"
)

// 2.1 转义字符
func EscapedCharacters() {
	fmt.Println("\"hello world\"")
}

const (
	Version int = 10
)

// 2.2 变量与常量
func VariablesAndConstant() {
	var v1 int
	v1 = 1
	var v2 int = 2
	var v3 = 3
	v4 := 4
	fmt.Printf("v1=%v, v2=%v, v3=%v, v4=%v\n", v1, v2, v3, v4)

	var (
		v5     = 5
		v6 int = 6
	)
	fmt.Printf("v5=%v, v6=%v\n", v5, v6)

	const (
		c1 = 8
		c2 = iota
		c3
	)
	fmt.Printf("c1=%v, c2=%v, c3=%v\n", c1, c2, c3)
}

// 2.5 fmt格式

func FmtVerbs() {
	i := 65
	fmt.Printf("i=%d, Type=%T\n", i, i)
	fmt.Printf("%d, %b, %o, %x, %X, %c, %q\n", i, i, i, i, i, i, i)

	f := 3.14
	fmt.Printf("%.2f\n", f)
	fmt.Printf("%.10f\n", f)
	fmt.Printf("%.ef\n", f)

	b := false
	fmt.Printf("%t\n", b)

	s := "hello world"
	fmt.Printf("%s\n", s)
	fmt.Printf("%q\n", s)

	p := &s
	fmt.Printf("%p\n", p)
}

// 3.1 if else
func IfElse() {
	var age uint8
	fmt.Scanln(&age)
	if age < 13 {
		fmt.Println("age<13")
	} else {
		fmt.Println("age>=13")
	}
}

// 3.2 switch case
func SwitchCase() {
	var weekday uint8
	fmt.Println("input weekday")
	fmt.Scanln(&weekday)
	switch weekday {
	case 1:
		fmt.Println("星期一")
	case 2:
		fmt.Println("星期二")
	default:
		fmt.Println("输入有误")
	}
}

// 3.3 for
func For() {
	i := 0
	for {
		if i == 10 {
			break
		}
		i++
		fmt.Print(i, "\t")
	}
	fmt.Println()

	j := 0
	for j < 10 {
		j++
		fmt.Print(j, "\t")
	}
	fmt.Println()

	for i := 0; i < 10; i++ {
		fmt.Print(i, "\t")
	}
	fmt.Println()
}

// 3.4 label
func LabelAndGoto() {
outside:
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			fmt.Print("+ ")
			if i == 5 && j == 4 {
				break outside
			}
		}
		fmt.Println()
	}
}

// 3.5 function
func getSum(a, b int) (sum, diff int) {
	sum = a + b
	diff = a - b
	return
}

func Function() {
	res1, res2 := getSum(2, 3)
	fmt.Println("res1=", res1, " res2=", res2)
	// fmt.Printf("%v, %T", getSum, getSum)

	// 函数也是一种数据类型
	var getRes = func(a, b int) (sum, diff int) {
		sum = a + b
		diff = a - b
		return
	}

	res1, res2 = getRes(2, 3)
	fmt.Println("res1=", res1, " res2=", res2)

	// 匿名函数
	res1, res2 = func(a, b int) (sum, diff int) {
		sum = a + b
		diff = a - b
		return
	}(6, 7)
	fmt.Println("res1=", res1, " res2=", res2)
}

// 3.6 defer
func deferUtil() func(int) int {
	i := 0
	return func(n int) int {
		fmt.Printf("本次调用接收到n=%v\n", n)
		i++
		fmt.Printf("匿名工具函数被第%v次调用\n", i)
		return i
	}
}
func Defer() int {
	f := deferUtil()
	defer f(10)
	defer f(20)
	defer f(30)
	return f(40)
}

func DeferRecover() {
	defer func() {
		err := recover()
		if err != nil {
			fmt.Println(err)
		}
	}()
	n := 0
	fmt.Println(3 / n)
}

// 4.2 切片
func Slice() {
	s := make([]int, 3, 5)
	fmt.Printf("array=%v, len=%d, cap=%d\n", s, len(s), cap(s))

	s1 := []int{1, 2, 3}
	fmt.Printf("array=%v, len=%d, cap=%d\n", s1, len(s1), cap(s1))

	array := [3]int{1, 2, 3}
	s2 := array[:]
	fmt.Println("s2=", s2)
	s2 = append(s2, 4, 5, 6)
	fmt.Println("array=", array)
	fmt.Println("s2=", s2)

	s3 := append(s1, s2...)
	fmt.Println("s3=", s3)

	s4 := make([]int, 2)
	copy(s4, s3)
	fmt.Println("s4=", s4)

	str := "hello"
	fmt.Println("str=", str)
	for i, v := range str {
		fmt.Printf("str[%v]=%c\n", i, v)
	}

	// key := util.SelectByKey("注册", "登录")
	// fmt.Println("key: ", key)
}

func Copy() {
	s := []int{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
	fmt.Println(s)
	s1 := s[5:8]
	fmt.Println(s1)
	n := copy(s[4:], s1)
	fmt.Println(s)
	fmt.Println(s1)
	fmt.Println(n)

	s2 := make([]int, 6)
	n = copy(s2, s)
	fmt.Println(n, s2)

	// 直接从字符串中复制数据到[]byte
	b := make([]byte, 3)
	n = copy(b, "abcde")
	fmt.Println(n, b)
}

// 4.3 map
func Map() {
	var m1 map[string]string
	m1 = make(map[string]string)
	m1["a"] = "kavin"
	m1["b"] = "jack"
	m1["c"] = "lisa"
	fmt.Println("m1=", m1)
	for key, value := range m1 {
		fmt.Printf("key=%v, value=%v\n", key, value)
	}

	if v, ok := m1["b"]; ok {
		fmt.Println("m1[b]=", v)
	}
}

// bytes.Buffer
func BytesBuffer() {
	var b bytes.Buffer
	b.Grow(100)
	for i := 0; i < 100; i++ {
		b.WriteString("a")
	}
	s := b.String()
	fmt.Println(s)
}

// Unicode
func UnicodeAndRune() {
	r := '我'
	fmt.Printf("%T\n", r)
}

func Array() {
	a := [2]int{1, 2}
	var b [2]int
	b = a
	fmt.Println(a, b)
	b[0] = 100
	fmt.Println(a, b)
}

type point struct {
	a byte
	b byte
	c []int
}

type User struct {
	Name string
	Id   uint32
}

type Account struct {
	User
	password string
}

type Contact struct {
	*User
	Remark string
}

// 4.5 结构体
func Struct() {
	v := point{}
	fmt.Printf("%d\n", unsafe.Sizeof(v))
	type mesType uint16
	var u1000 uint16 = 1000
	var textMsg mesType = mesType(u1000)
	fmt.Printf("textMsg=%v, Type=%T\n", textMsg, textMsg)

	var u1 = User{
		Name: "kavin",
		Id:   100,
	}

	var u2 *User = &u1
	u2.Id = 456

	fmt.Printf("%p\n", u2)

	var a1 = Account{
		User: User{
			Name: "kavin",
			Id:   123,
		},
		password: "12345",
	}

	fmt.Println(a1)

	var c1 *Contact = &Contact{
		User:   u2,
		Remark: "789",
	}
	c1.Name = "jack"
	fmt.Println(*c1)
}

func (u User) printName() {
	fmt.Println("name=", u.Name)
}

func (u *User) setId(id uint32) {
	u.Id = id
}

func Method() {
	u := User{
		Name: "kavin",
		Id:   12,
	}
	fmt.Println(u)
	u.printName()
	u.setId(23)
	fmt.Println(u)
}

// 5.2 接口
type textMes struct {
	Type string
	Text string
}

func (tm *textMes) setText() {
	tm.Text = "一段文字"
}

func (tm *textMes) setType() {
	tm.Type = "文字消息"
}

type imgMes struct {
	Type string
	Img  string
}

func (im *imgMes) setImg() {
	im.Img = "一张图片"
}

func (im *imgMes) setType() {
	im.Type = "图片消息"
}

// 声明一个接口统一管理
// 接口里面存的是需要用接口来调用的方法
type Mes interface {
	setType()
}

// 我们希望SendMes即能填textMes，又能填imgMes
// 希望可以根据传入的类型自动调用对应的setType
// 接口里面保存的是：值+类型
func SendMes(m Mes) {
	// 函数名一样的话就可以直接调用
	m.setType()

	// 用switch case来实现不同类型调用不同的函数，针对函数名不一样的情况
	switch mptr := m.(type) {
	case *textMes:
		mptr.setText()
	case *imgMes:
		mptr.setImg()
	}

	fmt.Println("m=", m)
}

func Interface() {
	tm := textMes{}
	SendMes(&tm)

	im := imgMes{}
	SendMes(&im)

	// 空接口
	var n1 = 1
	n1interface := interface{}(n1)

	// 类型断言：把空接口还原成原始类型
	if n2, ok := n1interface.(int); ok {
		fmt.Println(n2)
	} else {
		fmt.Println("类型断言失败")
	}
}

// 5.3 协程
// 主线程结束时，协程会被中断，需要有效的阻塞机制
var (
	c    int
	lock sync.Mutex
)

func PrimeNum(n int) {
	for i := 2; i < n; i++ {
		if n%i == 0 {
			return
		}
	}
	fmt.Printf("%v\t", n)
	lock.Lock()
	c++
	lock.Unlock()
}

func Goroutine() {
	for i := 2; i < 100001; i++ {
		go PrimeNum(i)
	}
	var key string
	fmt.Scanln(&key)
	fmt.Printf("\n共找到%v个素数\n", c)
}

// 5.4 channel
func pushNum(c chan int) {
	for i := 0; i < 100; i++ {
		c <- i
	}
	close(c)
}

func pushPrimeNum(n int, c chan int) {
	for i := 2; i < n; i++ {
		if n%i == 0 {
			return
		}
	}
	c <- n
}

func Channel() {
	var c1 chan int = make(chan int, 100)
	go pushNum(c1)
	// for {
	// 	v, ok := <-c1
	// 	if ok {
	// 		fmt.Printf("%v\t", v)
	// 	} else {
	// 		break
	// 	}
	// }
	for v := range c1 {
		fmt.Printf("%v\t", v)
	}

	var c2 chan int = make(chan int, 100)
	for i := 0; i < 100001; i++ {
		go pushPrimeNum(i, c2)
	}
Print:
	for {
		select {
		case v := <-c2:
			fmt.Printf("%v\t", v)
		default:
			fmt.Println("所有素数已经找到")
			break Print
		}
	}
	fmt.Println("end")
}
