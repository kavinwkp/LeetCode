package main

import (
	"fmt"
	"log"
	"os"
	"time"

	"gorm.io/driver/mysql"
	"gorm.io/gorm"
	"gorm.io/gorm/logger"
)

var db *gorm.DB

func init() {
	newLogger := logger.New(
		log.New(os.Stdout, "\r\n", log.LstdFlags), // io writer
		logger.Config{
			SlowThreshold: time.Second, // 慢 SQL 阈值
			LogLevel:      logger.Info, // Log level
			Colorful:      true,        // 禁用彩色打印
		},
	)
	dsn := "root:111@tcp(127.0.0.1:3306)/school?charset=utf8mb4&parseTime=True&loc=Local"
	// d, err := gorm.Open(mysql.Open(dsn), &gorm.Config{Logger: logger.Default.LogMode(logger.Info)})
	d, err := gorm.Open(mysql.Open(dsn), &gorm.Config{
		Logger: newLogger,
	})
	if err != nil {
		log.Fatal(err)
	}
	db = d
}

type Company struct {
	ID   int
	Name string
}

// User 属于 Company CompanyID 是外键
type User struct {
	gorm.Model
	Name      string
	CompanyID int
	Company   Company `gorm:"constraint:OnUpdate:CASCADE,OnDelete:SET NULL;"`
}

func createTable() {
	// 数据库配置文件设置创建的表为InnoDB
	// default-storage-engine=INNODB
	db.AutoMigrate(&Company{}, &User{})
}

func main() {
	createTable()
	// db.Create(&Company{
	// 	Name: "BAT",
	// })
	// db.Create(&User{
	// 	Name:      "kavin",
	// 	CompanyID: 1,
	// })
	// db.Create(&User{
	// 	Name:      "jack",
	// 	CompanyID: 2,
	// })
	// db.Create(&User{
	// 	Name:      "lisa",
	// 	CompanyID: 1,
	// })
	// db.Delete(&User{}, 1)
	// db.Model(&User{}).Where("id = 4").Update("name", "jack2")
	// db.Delete(&Company{}, 2)
	// db.Where("Company_ID = 2").Delete(&User{})
	var users []User
	db.Find(&users)
	for _, v := range users {
		fmt.Println("ID:", v.ID, "Name:", v.Name, "CompanyID:", v.CompanyID)
	}
}
