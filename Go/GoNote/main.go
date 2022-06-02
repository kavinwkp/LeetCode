package main

import (
	"gonote/note"
	"unsafe"
)

func toString(bs []byte) string {
	return *(*string)(unsafe.Pointer(&bs))
}

func main() {
	// note.EscapedCharacters()
	// note.VariablesAndConstant()
	// fmt.Println(note.Version)
	// note.FmtVerbs()
	// note.IfElse()
	// note.SwitchCase()
	// note.For()
	// note.Function()
	// note.Defer()
	// note.DeferRecover()
	// fmt.Println("no panic")
	// note.Slice()
	// note.Copy()
	// note.Map()
	// note.BytesBuffer()
	// note.UnicodeAndRune()
	// note.Array()
	// note.Struct()
	// note.Method()
	// note.Interface()
	// note.Interface2()
	// note.Goroutine()
	// note.Channel()

	// note.RandNum()
	// note.StrConv()
	// note.PackageStrings()
	// note.MethodSet()
	// note.MethodExpression()
	// note.MethodValue()
	// note.PackageUtf8()
	// note.PackageTime()
	// note.FileOperation()
	// note.Errors()
	// fmt.Println(runtime.NumCPU())
	// note.Log()
	// note.FileReadAndWrite()
	// note.PackageJson()
	// note.Server()
	// note.Client()

	// note.InitDB()

	// redis
	note.RedisBasic()
}
