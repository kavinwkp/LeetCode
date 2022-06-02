package note

import (
	"fmt"

	"github.com/go-redis/redis"
)

func RedisBasic() {
	// redis.Options是redis包定义的结构体，用于设置redis连接
	opt := redis.Options{
		Addr:     "localhost:6379",
		Password: "",
		DB:       0,
	}
	db := redis.NewClient(&opt)
	err := db.Ping().Err()
	if err != nil {
		panic(err)
	}
	//context.Context 是context包定义的接口，代表一个上下文，包括有效期限、取消标志和其他跨api进程的值
	// 上下文接口定义的方法可以安全地在并发中使用，简称ctx
	// 上下文的作用是允许中断：任务中断后，处理器保存上下文，以便之后根据上下文在任务的同意位置继续执行
	// ctx := context.Background()
	// db.Do(ctx, "set", "k1", "v1")
	// res, err := db.Do(ctx, "get", "k1").Result()
	// if err != nil {
	// 	if err == redis.Nil {
	// 		fmt.Println("该key不存在")
	// 	}
	// 	panic(err)
	// }
	// fmt.Println("res=", res.(string))
	err = db.Set("name1", "jack", 0).Err()
	if err != nil {
		panic(err)
	}
	res := db.Get("name1").String()
	fmt.Println(res)
	// res := db.LRange("mylist", 0, -1).String()
	// fmt.Println(res)
}
