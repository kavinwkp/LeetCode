package util

import (
	"fmt"
	"os"
	"strings"
)

// 判断文件是否存在
func FileExist(path string) bool {
	fi, err := os.Stat(path)
	if err == nil {
		return !fi.IsDir() // 不是文件夹
	}
	return os.IsExist(err)
}

// 根据文件路径创建文件夹
func MkdirWithFilePath(filepath string) error {
	paths := strings.Split(filepath, "/")
	fmt.Println(paths)
	paths[len(paths)-1] = ""
	fmt.Println(paths)
	for i, v := range paths {
		if i == len(paths)-1 {
			break
		}
		if i != 0 {
			paths[len(paths)-1] += "/"
		}
		paths[len(paths)-1] += v
	}
	fmt.Println(paths[len(paths)-1])
	return os.MkdirAll(paths[len(paths)-1], 0775)
}
