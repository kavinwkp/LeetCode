package note

import (
	"bufio"
	"bytes"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"gonote/util"
	"io"
	"math/rand"
	"net"
	"os"
	"reflect"
	"strconv"
	"strings"
	"time"
	"unicode/utf8"

	"gorm.io/driver/mysql"
	"gorm.io/gorm"
)

// 6.1 随机数
func RandNum() {
	rand.Seed(time.Now().UnixNano())
	fmt.Println(rand.Intn(11))
}

// 6.2 字符串类型转换
func StrConv() {
	i := 123
	s := "hello"
	s2 := fmt.Sprintf("%d@%s", i, s)
	fmt.Println(s2)

	var (
		i1 int
		s1 string
	)
	n, err := fmt.Sscanf(s2, "%d@%s", &i1, &s1)
	if err != nil {
		panic(err)
	}
	fmt.Printf("解析了%d个数据\n", n)
	fmt.Println(i1, s1)

	s4 := strconv.FormatInt(65, 16)
	fmt.Println(s4)
	u1, err := strconv.ParseUint(s4, 16, 0)
	if err != nil {
		panic(err)
	}
	fmt.Println("u1=", u1)
}

// 6.3 strings包函数
func PackageStrings() {
	fmt.Println(strings.Contains("hello", "ll"))
	fmt.Println(strings.Count("hello world", "l"))
	fmt.Println(strings.Index("hello", "ll"))
	fmt.Println(strings.Replace("hello", "l", "x", -1))
	fmt.Println(strings.Repeat("hi", 3))
	fmt.Println(strings.HasPrefix("hello", "he"))
	fmt.Println(strings.HasSuffix("hello", "lo"))
	fmt.Println(strings.EqualFold("hello", "HeLLo"))
	fmt.Println(strings.ToLower("HELLO"))
	fmt.Println(strings.ToUpper("hello"))

	// 按空白字符分隔字符串
	fmt.Println(strings.Fields("hello world\n kavin\t jack"))
	// 按指定的字符串拆分
	fmt.Println(strings.Split("hello-world-kavin-jack", "-"))
	// 保留在结尾
	fmt.Println(strings.SplitAfter("hello-world-kavin-jack", "-"))

	// 修建字符串两边
	fmt.Println(strings.Trim("#*www.baidu.com $*#", "#* $"))

	// 清除两边的空白字符
	fmt.Println(strings.TrimSpace("\n\twww.baidu.com\t\r\n"))
}

type S struct{}

type T struct {
	S
}

func (S) sVal()  {}
func (*S) sPtr() {}
func (T) tVal()  {}
func (*T) tPtr() {}

func methodSet(a interface{}) {
	t := reflect.TypeOf(a)
	fmt.Println(t.NumMethod())
	for i, n := 0, t.NumMethod(); i < n; i++ {
		fmt.Println(i)
		m := t.Method(i)
		fmt.Println(m.Name, m.Type)
	}
}

func MethodSet() {
	var t T

	methodSet(t)
	// fmt.Println("---------------")
	// methodSet(&t)
}

type N int

func (n *N) test() {
	fmt.Printf("test.n: %p, %d\n", n, *n)
}

// func MethodExpression() {
// 	var n N = 25
// 	fmt.Printf("main.n: %p, %d\n", &n, n)

// 	f1 := N.test
// 	f1(n)

// 	f2 := (*N).test
// 	f2(&n)
// }

// func call(m func()) {
// 	m()
// }

func MethodValue() {
	var n N = 100
	p := &n

	n++
	f1 := n.test

	n++
	f2 := p.test

	n++
	fmt.Printf("main.n: %p, %d\n", p, n)

	f1()
	f2()

	// fmt.Printf("main.n: %p, %d\n", p, n)
	// n++
	// call(n.test)

	// n++
	// call(n.test)

}

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

func Interface2() {
	var s Sleeper
	dog := Dog{Name: "小白"}
	cat := Cat{Name: "kitty"}
	s = dog
	AnimalSleep(s)

	s = cat
	AnimalSleep(s)

	// sleepList := []Sleeper{
	// 	Dog{Name: "小黑"},
	// 	Cat{Name: "Kitty"},
	// }

	// for _, s := range sleepList {
	// 	s.Sleep()
	// }

	sleepList := []LazyAnimal{
		Dog{Name: "小黑"},
		Cat{Name: "Kitty"},
	}

	for _, s := range sleepList {
		s.Sleep()
		s.Eat("food")
	}
}

// 6.4 中文字符常见操作
func PackageUtf8() {
	str := "hello,世界"
	fmt.Println(utf8.RuneCountInString(str))
	fmt.Println(utf8.ValidString(str[:len(str)-1]))
}

// 6.5 time
func PackageTime() {
	for i := 0; i < 5; i++ {
		fmt.Printf(".")
		time.Sleep(time.Millisecond * 100)
	}
	fmt.Println()

	d1, err := time.ParseDuration("1000s")
	if err != nil {
		panic(err)
	}
	fmt.Println("d1=", d1)

	t1, err := time.Parse("2006年1月2日, 15时4分", "2022年5月23日, 20时18分")
	if err != nil {
		panic(err)
	}
	fmt.Println(time.Since(t1))
}

// 6.6 文件常见操作
func FileOperation() {
	// util.MkdirWithFilePath("d1/d2/file")	// 创建文件夹
	dirEntrys, err := os.ReadDir("/home/kavin/Documents/GoProjects/GoNote")
	if err != nil {
		panic(err)
	}
	for _, v := range dirEntrys {
		fmt.Println(v.Name())
	}

	file, err := os.OpenFile("f1", os.O_RDWR|os.O_CREATE, 0666)
	defer file.Close()
	data, err := os.ReadFile("f1")
	if err != nil {
		panic(err)
	}
	fmt.Println("f1中的数据为", string(data))
	err = os.WriteFile("f2", data, 0775)
	if err != nil {
		panic(err)
	}

}

// 6.7 文件读写
func FileReadAndWrite() {
	f5, err := os.OpenFile("f5", os.O_WRONLY|os.O_CREATE, 0666)
	if err != nil {
		panic(err)
	}
	defer f5.Close()
	writer := bufio.NewWriter(f5)
	fmt.Println(writer.Size())
	// for i := 0; i < 4; i++ {
	// 	fileName := fmt.Sprintf("f%v", i)
	// 	data, err := os.ReadFile(fileName)
	// 	if err != nil {
	// 		panic(err)
	// 	}
	// 	data = append(data, '\n')
	// 	writer.Write(data) // 写入缓冲区
	// }
	// writer.Flush() // 写入硬盘
}

// 6.8 错误
func Errors() {
	// defer func() {
	// 	err := recover()
	// 	fmt.Println("捕捉到了错误:", err)
	// }()
	// err1 := errors.New("出错了")
	// fmt.Println("err1=", err1)
	// panic(err1)
	err1 := errors.New("new error")
	err2 := fmt.Errorf("err2: [%w]", err1)
	fmt.Println(err2)
	err3 := errors.Unwrap(err2)
	fmt.Printf("err1:%#v\n", err1)
	fmt.Printf("err3:%#v\n", err3)
	fmt.Println(err1 == err3)
	err4 := errors.New("new error")
	fmt.Printf("err4:%#v\n", err4)
	fmt.Println(err1 == err4)
	fmt.Println(err3 == err4)

	fmt.Println(errors.Is(err2, err1))
	fmt.Println(errors.Is(err2, err3))
	fmt.Println(errors.Is(err1, err3))
	fmt.Println(errors.Is(err3, err1))
	fmt.Println()
	fmt.Println(errors.Is(err2, err4))
	fmt.Println(errors.Is(err1, err4))
}

// 6.9 日志
func Log() {
	defer func() {
		err := recover()
		fmt.Println("捕捉到了错误:", err)
	}()
	err := errors.New("出错了")
	util.INFO.Println(err)
	// util.WARN.Panicln(err)
	util.ERR.Fatalln(err)
}

// 6.10 单元测试
func IsNotNegative(n int) bool {
	return n > -1
}

// 9.1 json
func PackageJson() {
	type user struct {
		Name  string `json:"name"`
		Age   int
		Email string `json:"email,omitempty"` // 没有值就跳过
		Job   map[string]string
	}
	u1 := user{
		Name: "kavin",
		Age:  23,
		Job: map[string]string{
			"早上": "送外卖",
			"中午": "写代码",
		},
	}
	data, _ := json.Marshal(&u1) // 编码，可以直接传地址
	fmt.Println(string(data))
	buf := new(bytes.Buffer)
	json.Indent(buf, data, "", "\t")
	fmt.Println(buf.String())

	var u2 user
	json.Unmarshal(data, &u2) // 解码
	fmt.Println("u2=", u2)
}

// 10.1 TCP
func Client() {
	conn, err := net.Dial("tcp", "127.0.0.1:2022")
	if err != nil {
		fmt.Println("拨号失败")
		return
	}
	defer conn.Close()
	for {
		mes := struct {
			UserName string
			Mes      string
		}{
			UserName: "kavin",
		}
		fmt.Println("请输入要发送的内容")
		fmt.Scanf("%s\n", &mes.Mes)
		if mes.Mes == "" {
			fmt.Println("输入内容为空")
			return
		}
		if mes.Mes == "exit" {
			return
		}
		// data, _ := json.Marshal(&mes)
		// n, err := conn.Write(data)
		// if err != nil {
		// 	fmt.Println("发送失败")
		// 	return
		// }
		// fmt.Printf("成功发送了%v个字节", n)
		err := json.NewEncoder(conn).Encode(&mes)
		if err != nil {
			fmt.Println("发送失败")
			return
		}
	}
}

func Server() {
	listener, err := net.Listen("tcp", ":2022")
	if err != nil {
		fmt.Println("监听失败:", err)
		return
	}
	defer listener.Close()
	for {
		fmt.Println("主进程等待客户端连接...")
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("监听失败")
			continue
		}
		go func(conn net.Conn) {
			fmt.Println("已开启一个协程")
			defer conn.Close()
			for {
				mes := struct {
					UserName string
					Mes      string
				}{}
				// buf := make([]byte, 4096)
				// n, err := conn.Read(buf)
				// if err == io.EOF {
				// 	fmt.Println("客户端退出")
				// 	return
				// }
				// if err != nil {
				// 	fmt.Println("读取消息失败")
				// 	return
				// }
				// json.Unmarshal(buf[:n], &mes)
				err := json.NewDecoder(conn).Decode(&mes)
				if err == io.EOF {
					fmt.Println("客户端退出")
					return
				}
				if err != nil {
					fmt.Println("读取消息失败")
					return
				}
				fmt.Printf("%s 说: %s\n", mes.UserName, mes.Mes)
			}
		}(conn)
	}
}

var db *sql.DB

type Animal struct {
	ID   int64
	Name string
	Age  int64
}

func InitDB() {
	dsn := "root:111@(127.0.0.1:3306)/school?charset=utf8mb4&parseTime=True&loc=Local"
	db, err := gorm.Open(mysql.Open(dsn), &gorm.Config{})
	if err != nil {
		fmt.Println("connect db error", err)
	}
	fmt.Println(db)
	query_test(db)
}

func query_test(db *gorm.DB) error {
	// 根据主键查询第一条记录
	var animal Animal
	db.First(&animal)
	fmt.Println(animal)

	// 根据主键查询最后一条记录
	var animal2 Animal
	db.Last(&animal2)
	fmt.Println(animal2)

	// 指定某条记录（仅当主键为整型时可用）
	var animal3 Animal
	db.First(&animal3, 2)
	fmt.Println(animal3)

	// where条件
	// 符合条件的第一条记录
	var animal4 Animal
	db.Where("name = ?", "jim").First(&animal4)
	fmt.Println("where : ", animal4, animal4.ID, animal4.Name, animal4.Age)

	// 符合条件的所有记录
	var animals5 []Animal
	db.Where("name = ?", "jim").Find(&animals5)
	fmt.Println(animals5)
	for k, v := range animals5 {
		fmt.Println("k:", k, "ID:", v.ID, "Name:", v.Name, "Age:", v.Age)
	}

	// IN
	var animals6 []Animal
	db.Where("name IN (?)", []string{"demo-test", "demotest"}).Find(&animals6)
	fmt.Println(animals6)

	// LIKE
	var animals7 []Animal
	db.Where("name like ?", "jim%").Find(&animals7)
	fmt.Println(animals7)

	// AND
	var animals8 []Animal
	db.Where("name = ? AND age >= ?", "jim", "24").Find(&animals8)
	fmt.Println(animals8)

	// 总数
	var count int64
	var animals9 []Animal
	db.Where("name = ?", "galeone").Or("name = ?", "jim").Find(&animals9).Count(&count)
	fmt.Println(animals9)
	fmt.Println(count)

	//原生查询，select all
	var animals10 []Animal
	db.Raw("SELECT id, name, age From animals WHERE name = ? AND age = ? ", "galeone", "30").Scan(&animals10)
	fmt.Println("Scan: ", animals10)

	return nil
}
