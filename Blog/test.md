## python
```py
#!/usr/bin/env python3
```
使用哪个解释器去解释/运行脚本

## chmod
```bash
7 = 4 + 2 + 1
-r(可读)w(可写)x(可执行)-
```
分为三个部分:
+ Owner
+ Group
+ Others

```bash
644 -rw-r--r-- 所有者可读可写
755 -rwxr-xr-x 所有者可读可写可执行
777 -rwxrwxrwx
```

## 文件基本操作

mv: 移动文件，重命名
```bash
mv hwllo.txt hello.txt  # 重命名
mv dir1 dir2            # 移动文件夹
```

cp: 复制文件
```bash
cp a.txt a_copy.txt
cp -r dir1 dir2     # 复制文件夹
```

rm: 删除文件，没有回收站
```bash
rm a.txt        # 删除单个文件
rm a.txt b.txt c.txt    # 删除多个文件
rm -r dir1      # 删除文件夹
```

## 环境变量

$PATH: 以`:`分割的文件夹列表
指明到哪里去找可执行文件(按照文件夹的顺序查找)

```bash
PATH=$PATH:$PWD     # 把当前目录追加到PATH中
```
只对当前终端有效
要对全局有效的话要加入到`.bashrc`或`.zshrc`