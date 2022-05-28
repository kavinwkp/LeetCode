## Git的正确使用姿势与最佳实践

1. 学习基本的Git命令，并了解原理，在遇到Git相关问题时，能自行排查并解决
2. 了解研发流程中的基本概念和规范，学会正确使用Git

## Git基本使用方式

项目初始化

mkdir demo
cd demo
git init

git init参数
--initial-brance 初始化的分支，默认是master分支
--bare 创建一个裸仓库（纯git目录，没有工作目录）
--template 可以通过模板来创建预先构建好的自定义git目录

╰─ tree .git
.git
├── branches
├── config
├── description
├── HEAD
├── hooks
│   ├── applypatch-msg.sample
│   ├── commit-msg.sample
│   ├── fsmonitor-watchman.sample
│   ├── post-update.sample
│   ├── pre-applypatch.sample
│   ├── pre-commit.sample
│   ├── prepare-commit-msg.sample
│   ├── pre-push.sample
│   ├── pre-rebase.sample
│   ├── pre-receive.sample
│   └── update.sample
├── info
│   └── exclude
├── objects
│   ├── info
│   └── pack
└── refs
    ├── heads
    └── tags

- config: git配置
- HEAD: 指向当前的分支 ref: refs/heads/master
- objects: 文件信息
- refs: 分支信息

+ 工作区：本地的文件
+ 暂存区：通过git add将文件加入到暂存区
+ Git仓库：通过git commit提交到远程仓库

#### 2.1.1 Git Config

--system: 保存在 /etc/gitconfig 中
--global: 保存在 ~/.gitconfig 中
--local: 保存在 .git/config 中

每个级别的配置可能重复，但是低级别的配置会覆盖高级别的配置

**用户名配置**

git config --global user.name "kavin"
git config --global user.email 1817136760@qq.com

**instead of配置**

将https协议替换成ssh协议

git config --global url.git@github.com:.insteadOf https://github.com

**Git命令别名配置**

git config --global alias.cin "commit --amend --no-edit"

### 2.2 Git Remote

本地和远程仓库的关联信息

查看Remote
git remote -v

刚开始没有，需要添加

git remote add origin_ssh git@github.com:git/git.git
git remote add origin_http https://github.com/git/git.git

再次查看

╰─ git remote -v
origin_http	https://github.com/git/git.git (fetch)
origin_http	https://github.com/git/git.git (push)
origin_ssh	git@github.com:git/git.git (fetch)
origin_ssh	git@github.com:git/git.git (push)

config文件也会有相应变化，多了origin_ssh和origin_http

╰─ cat .git/config 
[core]
	repositoryformatversion = 0
	filemode = true
	bare = false
	logallrefupdates = true
[remote "origin_ssh"]
	url = git@github.com:git/git.git
	fetch = +refs/heads/*:refs/remotes/origin_ssh/*
[remote "origin_http"]
	url = https://github.com/git/git.git
	fetch = +refs/heads/*:refs/remotes/origin_http/*

#### 2.2.2 SSH Remote

URL: git@github.com:git/git.git

免密配置
SSH可以通过公私钥的机制，将生成公钥存放在服务端，从而实现免密访问

目前Key的类型有4种，分别是dsa、rsa、ecdsa、ed25519
默认使用的是rsa，由于一些安全问题，现在推荐使用ed25519

╰─ ssh-keygen -t ed25519 -C "1817136760@qq.com"
Generating public/private ed25519 key pair.
Enter file in which to save the key (/home/kavin/.ssh/id_ed25519): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/kavin/.ssh/id_ed25519.
Your public key has been saved in /home/kavin/.ssh/id_ed25519.pub.
The key fingerprint is:
SHA256:i0ZztEWiJGLtF60Pl4+Tw6fXq4Jmyn4HGGWxz40aer0 1817136760@qq.com
The key's randomart image is:
+--[ED25519 256]--+
|  o.. oo. .      |
| . ..o+o.o       |
|   . ooo...      |
|    o ++o=       |
|     ++=S+.      |
|    .o.BB.o      |
|    . =oo= .     |
|   . o= +.. .    |
|   .+= .Eo....   |
+----[SHA256]-----+

╰─ cat /home/kavin/.ssh/id_ed25519.pub 
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIBv8hu55tnA1l7b0XdJ7/WDAdBE9ZiCXN7S3JdWYfoIL 1817136760@qq.com

然后添加到github

### 2.3 Git Add

添加一个文件

╰─ vim README.md

查看当前状态，是红色的

╰─ git status
On branch master

No commits yet

Untracked files:
  (use "git add <file>..." to include in what will be committed)

	README.md

nothing added to commit but untracked files present (use "git add" to track)

执行git add .之后，就加入了暂存区

╰─ git add .

再次查看状态，变成绿色了

╰─ git status
On branch master

No commits yet

Changes to be committed:
  (use "git rm --cached <file>..." to unstage)

	new file:   README.md

在objects下生成一个文件

├── objects
│   ├── 3b
│   │   └── 18e512dba79e4c8300dd08aeb37f8e728b8dad

可以查看该文件

╰─ git cat-file -p 3b18e512dba79e4c8300dd08aeb37f8e728b8dad
hello world

保存的就是新建的文件的内容

提交

╰─ git commit -m "add readme"
[master (root-commit) c18499f] add readme
 1 file changed, 1 insertion(+)
 create mode 100644 README.md

objects目录再次发生变化

├── objects
│   ├── 3b
│   │   └── 18e512dba79e4c8300dd08aeb37f8e728b8dad
│   ├── 43
│   │   └── b71c903ff52b9885bd36f3866324ef60e27b9b
│   ├── c1
│   │   └── 8499fff0da1411179060f661152f5b92e41cfe


同样可以查看，第一个文件保存的是目录树
╰─  git cat-file -p 43b71c903ff52b9885bd36f3866324ef60e27b9b
100644 blob 3b18e512dba79e4c8300dd08aeb37f8e728b8dad	README.md

第二个文件保存的是提交信息

╰─ git cat-file -p c18499fff0da1411179060f661152f5b92e41cfe
tree 43b71c903ff52b9885bd36f3866324ef60e27b9b
author kavinwkp <1817136760@qq.com> 1653704489 +0800
committer kavinwkp <1817136760@qq.com> 1653704489 +0800

add readme

可以从git log里面看到，两个commit序号是一样的

commit c18499fff0da1411179060f661152f5b92e41cfe (HEAD -> master)
Author: kavinwkp <1817136760@qq.com>
Date:   Sat May 28 10:21:29 2022 +0800

    add readme

### 2.5 Objects

commit/tree/blob在git里面都统一称为Object

- blob: 存储文件内容
- tree: 存储文件目录信息
- commit: 存储提交信息，一个commit可以对应唯一版本的代码

1、通过 commit 寻找到 tree 信息，每个 commit 都会存储相应的 tree id

╰─ git cat-file -p c18499fff0da1411179060f661152f5b92e41cfe
tree 43b71c903ff52b9885bd36f3866324ef60e27b9b
author kavinwkp <1817136760@qq.com> 1653704489 +0800
committer kavinwkp <1817136760@qq.com> 1653704489 +0800

add readme

2、通过 tree 存储的信息，获取到对应的目录树信息

╰─  git cat-file -p 43b71c903ff52b9885bd36f3866324ef60e27b9b
100644 blob 3b18e512dba79e4c8300dd08aeb37f8e728b8dad	README.md

3、从tree中获得blob的 id，通过blob id获取对应的文件内容

╰─ git cat-file -p 3b18e512dba79e4c8300dd08aeb37f8e728b8dad
hello world

### 2.6 Refs

refs 文件夹下也会发生变化

╰─ cat .git/refs/heads/master 
c18499fff0da1411179060f661152f5b92e41cfe

保存的是刚才提交的commit id

新建分支

╰─ git checkout -b test
Switched to a new branch 'test'

可以看到在refs/heads目录下生成了test文件

└── refs
    ├── heads
    │   ├── master
    │   └── test

其内容和 master 是一样的

╰─ cat .git/refs/heads/test 
c18499fff0da1411179060f661152f5b92e41cfe

refs的内容就是对应的commit id，因此把ref当做指针，指向对应的commit来表示当前ref对应的版本

**不同种类的ref**

refs/heads前缀表示的是分支，refs/tags前缀表示的是标签

分支一般用于开发截断，是可以不断添加commit进行迭代
标签一般表示的是一个稳定版本，指向的commit一般不会变更

通过git tag命令生成tag

╰─  git tag v0.0.1

可以看到tags目录下生成了v0.0.1的tag

└── refs
    ├── heads
    │   ├── master
    │   └── test
    └── tags
        └── v0.0.1

内容跟master是一样的

╰─ cat .git/refs/tags/v0.0.1 
c18499fff0da1411179060f661152f5b92e41cfe

### 2.7 Annotation Tag

附注标签：一种特殊的tag，可以给tag提供一些额外的信息

╰─  git tag -a v0.0.2 -m "add feature 1"

可以看到生成了v0.0.2标签

└── refs
    ├── heads
    │   ├── master
    │   └── test
    └── tags
        ├── v0.0.1
        └── v0.0.2

╰─ cat .git/refs/tags/v0.0.2 
20883e70ce48a660349d0ce58974a6d0113983a0

其内容指向了objects下新增的文件

├── objects
│   ├── 20
│   │   └── 883e70ce48a660349d0ce58974a6d0113983a0

查看其内容

╰─ git cat-file -p 20883e70ce48a660349d0ce58974a6d0113983a0
object c18499fff0da1411179060f661152f5b92e41cfe
type commit
tag v0.0.2
tagger kavinwkp <1817136760@qq.com> 1653707227 +0800

add feature 1

其中object表示它指向的真正的commit id，同时还有一些附加信息

### 2.8 追溯历史版本

+ 获取当前版本代码

通过ref指向的commit可以获取唯一的代码版本

+ 获取历史版本代码

commit里面会存有parent commit字段，通过commit的串联获取历史版本代码

再次修改README.md并提交
git add .
git commit -m "update readme"

通过git log查询到commit id

commit c0931534a6d62c4b924a9e52e3bf16c3a8055ef2 (HEAD -> test)
Author: kavinwkp <1817136760@qq.com>
Date:   Sat May 28 11:15:41 2022 +0800

    update readme

然后查看commit id的内容

╰─ git cat-file -p c0931534a6d62c4b924a9e52e3bf16c3a8055ef2
tree 3981df8931e1d31cc30e98366fddae7716919cf8
parent c18499fff0da1411179060f661152f5b92e41cfe
author kavinwkp <1817136760@qq.com> 1653707741 +0800
committer kavinwkp <1817136760@qq.com> 1653707741 +0800

update readme

通过parent字段可以知道当前commit的前一个commit

通过tree id查询到blob id，获取到文件内容
╰─ git cat-file -p 3981df8931e1d31cc30e98366fddae7716919cf8
100644 blob 759f3c31e352fe4917e59f548ec26b40e5771d4f	README.md

╰─ git cat-file -p 759f3c31e352fe4917e59f548ec26b40e5771d4f
#hello world

test也指向了最新的commit

╰─ cat .git/refs/heads/test 
c0931534a6d62c4b924a9e52e3bf16c3a8055ef2

### 2.9 修改历史版本

+ commit --amend

通过这个命令可以修改最近的一次commit信息，修改之后commit id会变

+ rebase

通过git rebase -i HEAD~3 可以实现对最近三个commit的修改

1. 合并commit
2. 修改具体的commit message
3. 删除某个commit

+ filter --branch

该命令可以指定删除所有提交中的某个文件或者全局修改邮箱地址等操作

使用git commit --amend 修改提交信息

然后git log，发现跟之前不一样了

commit aeb4d5e973f400acd452097bd7504141fcc40345 (HEAD -> test)
Author: kavinwkp <1817136760@qq.com>
Date:   Sat May 28 11:15:41 2022 +0800

    update readme!!!

可以看到commit信息发生了变化，正是刚才修改后的样子

╰─ git cat-file -p aeb4d5e973f400acd452097bd7504141fcc40345
tree 3981df8931e1d31cc30e98366fddae7716919cf8
parent c18499fff0da1411179060f661152f5b92e41cfe
author kavinwkp <1817136760@qq.com> 1653707741 +0800
committer kavinwkp <1817136760@qq.com> 1653709256 +0800

update readme!!!

但是tree和parent还是一样的

同时之前的那个commit还存在，只是没有refs指向它

╰─ git fsck --lost-found
Checking object directories: 100% (256/256), done.
dangling commit c0931534a6d62c4b924a9e52e3bf16c3a8055ef2

### 2.11 Git GC

通过git gc命令，可以删除一些不需要的object，以及会对object进行一些打包压缩来减少仓库的体积

reflog是用于记录操作日志，防止误操作后数据丢失，通过reflog来找到丢失的数据，手动将日志设置为过期

git gc prune=now指定的是修剪多久之前的对象，默认是两周前

╰─ git reflog expire --expire=now --all 

╰─ git gc --prune=now
Counting objects: 7, done.
Delta compression using up to 12 threads.
Compressing objects: 100% (3/3), done.
Writing objects: 100% (7/7), done.
Total 7 (delta 0), reused 0 (delta 0)


这样没有的commit就会被删掉了

╰─ git cat-file -p c0931534a6d62c4b924a9e52e3bf16c3a8055ef2
fatal: Not a valid object name c0931534a6d62c4b924a9e52e3bf16c3a8055ef2

### 2.13 Git Clone & Pull & Fetch

Clone
拉取完整的仓库到本地目录，可以指定分支，深度

Fetch
将远端某些分支最新代码拉取到本地，不会指定merge操作
会修改refs/remote内的分支信息，如果需要和本地代码合并需要手动操作

Pull
拉取远端某分支，并和本地代码进行合并，操作等同于git fetch + git merge
也可以通过 git pull --rebase完成
可能存在冲突，需要解决冲突

### 2.14 Git Push

将本地代码同步至远端
一般使用 git push origin main 命令

冲突问题
如果本地的commit记录和远端的commit历史不一致，则会发生冲突

如果该分支就自己一个人使用，或者团队内确认过可以修改历史则可以通过git push origin main -f来完成强制推送，一般不推荐主干分支进行该操作，正常都应该解决冲突后再进行推送

推送规则限制
可以通过保护分支，来配置一些保护规则，防止误操作，或者一些不合规的操作出现，导致代码对视

### Git研发流程


╰─ git push origin feature
Counting objects: 3, done.
Writing objects: 100% (3/3), 254 bytes | 254.00 KiB/s, done.
Total 3 (delta 0), reused 0 (delta 0)
remote: 
remote: Create a pull request for 'feature' on GitHub by visiting:
remote:      https://github.com/kavinwkp/demo/pull/new/feature
remote: 
To github.com:kavinwkp/demo.git
 * [new branch]      feature -> feature

新建分支并提交之后，会生成一个链接，打开连接之后，可以pull request提交一个请求
确认没问题之后就可以 Merge pull request
这样远程仓库就进行了合并分支

再拉取到本地

╰─ git pull origin main
remote: Enumerating objects: 1, done.
remote: Counting objects: 100% (1/1), done.
remote: Total 1 (delta 0), reused 0 (delta 0), pack-reused 0
Unpacking objects: 100% (1/1), done.
From github.com:kavinwkp/demo
 * branch            main       -> FETCH_HEAD
   76b33cd..f0d5eb3  main       -> origin/main
Updating 76b33cd..f0d5eb3
Fast-forward
 README.md | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)

通过git log可以看到进行了一次合并

commit f0d5eb3189ffcb24215fd8a3feee198e863ea197 (HEAD -> main, origin/main)
Merge: 76b33cd 7c4849c
Author: kavinwkp <71803246+kavinwkp@users.noreply.github.com>
Date:   Sat May 28 14:34:08 2022 +0800

    Merge pull request #1 from kavinwkp/feature
    
    update readme


### 代码合并

- Fast-Forward

不会产生一个merge节点，合并后保持一个线性历史，如果target分支有了更新，则需要通过rebase操作更新source branch后才可以合入

比如新建分支test，然后做一个修改并提交，然后切换会main分支进行合并

╰─ git merge test --ff-only 
Updating f0d5eb3..a18575f
Fast-forward
 README.md | 1 +
 1 file changed, 1 insertion(+)

可以从git log看出，没有产生merge节点

commit a18575febae09bc14fec2b54aa243079ac3f6c79 (HEAD -> main, test)
Author: kavinwkp <1817136760@qq.com>
Date:   Sat May 28 14:44:40 2022 +0800

    test

- Three-Way Merge

三方合并，会产生一个新的merge节点

同样新建分支test，然后做一个修改并提交，然后切换会main分支进行合并

╰─ git merge test --no-ff 
Merge made by the 'recursive' strategy.
 README.md | 1 +
 1 file changed, 1 insertion(+)

可以从git log看出增加了merge节点，跟pull request是一样的

commit 2d9ba4e378b8e4a5c49298fb908566ad953a6eaa (HEAD -> main)
Merge: a18575f 23ab11b
Author: kavinwkp <1817136760@qq.com>
Date:   Sat May 28 14:49:49 2022 +0800

    Merge branch 'test' into main


























