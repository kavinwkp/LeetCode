

## 寄存器

寄存器是`CPU`内部的构造，主要用于信息的存储。除此之外，`CPU`内部还有运算器，负责处理数据；控制器控制其他组件；外部总线连接`CPU`和各种组件，进行数据传输；内部总线负责`CPU`内部各种组件的数据处理

**为什么会出现寄存器？**

程序在内存中装载，由`CPU`来运行，`CPU`的主要职责就是用来处理数据。这个过程涉及到从存储器读取和写入数据，如果还要从内存中读取就太麻烦了，所以直接在`CPU`中开辟一些空间来存储数据，读取速度也比较快


### 认识寄存器

`CPU`从逻辑上可以分为3个模块，分别是控制单元、运算单元和存储单元，这三部分由`CPU`内部总线连接起来。

几乎所有的冯·诺伊曼型计算机的`CPU`，其工作都可以分为5个阶段：<font color=LightSeaGreen>取指令、指令译码、执行指令、访存取数、结果写回</font>

>1. 取指令阶段是将内存中的指令读取到`CPU`中寄存器的过程，程序寄存器用于存储下一条指令所在的地址
>2. 指令译码阶段，在取指令完成后，立马进入指令译码阶段，指令译码器按照预定的指令格式，对取回的指令进行拆分和解释，识别区分出不同的指令类别以及各种获取操作数的方法
>3. 执行指令阶段，译码完成后，就需要执行这一条指令了，此阶段的任务是完成指令所规定的各种操作，具体实现指令的功能
>4. 访问取数阶段，根据指令的需要，有可能需要从内存中提取数据，此阶段的任务是：根据指令地址码，得到操作数在内存中的地址，并从内存中读取该操作数用于运算
>5. 结果写回阶段（Write Back，WB），把运行结果写回到`CPU`内部寄存器中，以便被后续的指令快速地存取

###　计算机架构中的寄存器

{% g %}
寄存器是一块速度非常快的计算机内存
{% endg %}

在`8086`架构中，所有的内部寄存器、内部以及外部总线都是`16`位宽，可以存储两个字节，`8086`处理器有`14`个寄存器，每个寄存器都有一个特有的名称，即

>AX，BX，CX，DX，SP，BP，SI，DI，IP，FLAG，CS，DS，SS，ES

这`14`个寄存器有可能进行具体的划分，按照功能可以分为三种

>+ 通用寄存器
>+ 控制寄存器
>+ 段寄存器

### 通用寄存器

通用寄存器主要有四种 ，即<font color=LightSeaGreen>AX、BX、CX、DX</font>，同样的，这四个寄存器也是`16`位的，能存放两个字节，各自的功能为

>+ AX(Accumulator Register) ：累加寄存器，用于输入/输出和大规模的指令运算
>+ BX(Base Register)：基址寄存器，用来存储基础访问地址
>+ CX(Count Register)：计数寄存器，在迭代的操作中循环计数
>+ DX(data Register)：数据寄存器，也用于输入/输出操作。它还与 AX 寄存器一起使用，用于大数值的乘法和除法运算

这四种寄存器可以分为上半部分和下半部分，分成高位和低位两个`8`位数据寄存器
>`AX`寄存器可以分为两个独立的`8`位的`AH`和`AL`寄存器，另外3个类似

寄存器的存储方式是先存储低位，如果低位满足不了就存储高位，如果低位能够满足，高位用0补全

`8086 CPU`可以一次存储两种类型的数据

>+ 字节(byte)：一个字节由`8 bit`组成，这是一种恒定不变的存储方式
>+ 字(word)：字是由指令集或处理器硬件作为单元处理的固定大小的数据，对于`intel`来说，一个字长就是两个字节，字是计算机一个非常重要的特征，针对不同的指令集架构来说，计算机一次处理的数据也是不同的。有字（16位）、双字（32位）、四字（64位）等

#### AX寄存器

<font color=LightSeaGreen><b>累加寄存器</b></font>或者简称为累加器

```cpp
mov AX,20   // 将 20 送入寄存器 AX
mov AH,80   // 将 80 送入寄存器 AH
add AX,10   // 将寄存器 AX 中的数值加上 10
```
支持`MUL`和`DIV`指令

#### BX寄存器

<font color=LightSeaGreen><b>数据寄存器</b></font>，能够暂存一般数据，还用于<font color=LightSeaGreen>寻址</font>，即寻找物理内存地址。`BX`寄存器中存放的数据一般是作为<font color=LightSeaGreen>偏移地址</font>使用的。偏移地址是在段寄存器中存储的

#### CX寄存器

`CX`也是数据寄存器，能够暂存一般性数据。当在汇编指令中使用循环`LOOP`指令时，可以通过`CX`来指定需要循环的次数，每次执行循环`LOOP`时候，`CPU`会做两件事
+ 一是计数器自动减 1
+ 二是判断`CX`中的值，如果值为`0`则会跳出循环，如果值为`0`，则会继续执行循环

#### DX寄存器

`DX`也是数据寄存器，能够暂存一般性数据，支持`MUL`和`DIV`指令，同时也支持数值溢出等

### 段寄存器

>+ <font color=LightCoral>CS(Code Segment)：代码寄存器</font>，程序代码的基础位置
>+ DS(Data Segment)：数据寄存器，变量的基本位置
>+ SS(Stack Segment)：栈寄存器，栈的基础位置
>+ ES(Extra Segment)：其他寄存器，内存中变量的其他基本位置

### 索引寄存器

索引寄存器主要包含段地址的偏移量，分为

>BP(Base Pointer)：基础指针，它是栈寄存器上的偏移量，用来定位栈上变量
>SP(Stack Pointer): 栈指针，它是栈寄存器上的偏移量，用来定位栈顶
>SI(Source Index): 变址寄存器，用来拷贝源字符串
>DI(Destination Index): 目标变址寄存器，用来复制到目标字符串

### 状态和控制寄存器

+ <font color=LightCoral>IP(Instruction Pointer)：指令指针寄存器</font>，它是从`Code Segment`代码寄存器处的偏移来存储执行的下一条指令
+ FLAG：`Flag`寄存器用于存储当前进程的状态，这些状态有
    + 位置 (Direction)：用于数据块的传输方向，是向上传输还是向下传输
    + 中断标志位 (Interrupt) ：1 - 允许；0 - 禁止
    + 陷入位 (Trap) ：确定每条指令执行完成后，CPU 是否应该停止。1 - 开启，0 - 关闭
    + 进位 (Carry) : 设置最后一个无符号算术运算是否带有进位
    + 溢出 (Overflow) : 设置最后一个有符号运算是否溢出
    + 符号 (Sign) : 如果最后一次算术运算为负，则设置 1 =负，0 =正
    + 零位 (Zero) : 如果最后一次算术运算结果为零，1 = 零
    + 辅助进位 (Aux Carry) ：用于第三位到第四位的进位
    + 奇偶校验 (Parity) : 用于奇偶校验

### 物理地址

16 位的 CPU 指的是

+ CPU 内部的运算器一次最多能处理 16 位的数据

>运算器其实就是ALU，运算控制单元，它是CPU内部的三大核心器件之一，主要负责数据的运算

+ 寄存器的最大宽度为 16 位

>这个寄存器的最大宽度值就是通用寄存器能处理的二进制数的最大位数

+ 寄存器和运算器之间的通路为 16 位

>这个指的是寄存器和运算器之间的总线，一次能传输 16 位的数据

CPU 相关组件提供两个地址：<font color=LightSeaGreen>段地址和偏移地址</font>，这两个地址都是`16`位的，他们经由地址加法器变为`20`位的<font color=LightSeaGreen>物理地址</font>，这个地址即是输入输出控制电路传递给内存的物理地址，由此完成物理地址的转换

地址加法器采用`物理地址 = 段地址 * 16 + 偏移地址`的方法用段地址和偏移地址合成物理地址

段地址*16就是二进制左移4位，十六进制左移1位，作为基础地址，所以

{% g %}
物理地址 = 基础地址 + 偏移地址
{% endg %}

### 段寄存器

#### CS 寄存器

>CS（代码寄存器，存储段地址）和 IP（指令指针，存储偏移地址） 都是 8086 CPU 非常重要的寄存器，它们指出了 CPU 当前需要读取指令的地址

在 CPU 内部，由 CS 和 IP 提供段地址和偏移地址，由加法器负责转换为物理地址，输入输出控制电路负责输入/输出数据，指令缓冲器负责缓冲指令，指令执行器负责执行指令。在内存中有一段连续存储的区域，区域内部存储的是机器码、外面是地址和汇编指令

假设段地址和偏移地址分别是`2000`和`0000`，当这两个地址进入地址加法器后，会由地址加法器负责将这两个地址转换为物理地址`20000`，然后输入输出控制电路将`20`位的地址总线送到内存中，然后取出对应的数据，`B8 23 01`，`B8`是操作数，再送入指令缓存器中，此时这个指令就已经具备执行条件，然后 IP（指令指针）会自动增加。它会知道下一个需要读取指令的地址`0003`，在这之后，指令执行执行取出的`B8 23 01`这条指令，然后再把`2000`和`0003`送到地址加法器中再进行后续指令的读取，如此循环

总结一下 8086 CPU 的工作过程

>+ 段寄存器提供段地址和偏移地址给地址加法器
>+ 由地址加法器计算出物理地址通过输入输出控制电路将物理地址送到内存中
>+ 提取物理地址对应的指令，经由控制电路取回并送到指令缓存器中
>+ `IP`继续指向下一条指令的地址，同时指令执行器执行指令缓冲器中的指令