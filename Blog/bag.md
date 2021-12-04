## 背包问题

### 01背包

#### 动态规划五部曲

1、确定dp数组以及下标的含义

使用二维数组，`dp[i][j]`表示对于下标为`0~i`的物品，背包重量为`j`时的最大价值

<center>
<img src=https://github.com/kavinwkp/blogimage/raw/main/img/LeetCode/01/01bag-1.png width="60%">
</center>


2、确定递推公式

+ 不放物品`i`，则价值和`i-1`的一样，所以`dp[i][j] = dp[i-1][j]`
+ 放物品`i`，则背包重量要减少`weight[i]`，价值要增加`value[i]`，所以`dp[i][j] = dp[i-1][j-weight[i]] + value[i]`

如果放不进物品`i`，那只能是第一种，如果放得进，那就两种取较大值

3、dp数组初始化

第一列：首先背包重量为0时，一个物品都放不进来，所以第一列初始化为0
第一行：对于物品0，当背包重量小于物品0的重量时，都放不进来，所以初始化为0，当背包重量大于等于物品0的重量时，都放得进来，所以初始化为`value[0]`

剩下的初始化为0就行

<center>
<img src=https://github.com/kavinwkp/blogimage/raw/main/img/LeetCode/01/01bag-2.png width="60%">
</center>

4、确定遍历顺序

先遍历物品，再遍历背包，比较容易理解

也可以先遍历背包，再遍历物品，因为`dp[i][j]`来源于它的左上角

5、举例推导dp数组

对于物品1，背包重量为4时，可以选择不放物品1，价值不变，为15
如果选择放物品1，背包重量变为1，原先的背包重量为1的最大价值为15，再加上物品1的价值20，所以总价值为35，所以将`dp[1][4]`更新为35

<center>
<img src=https://github.com/kavinwkp/blogimage/raw/main/img/LeetCode/01/01bag-3.png width="60%">
</center>

#### 代码

```cpp
int main(int argc, char const *argv[]) {
    vector<int> weight = {1, 3, 4};
    vector<int> value = {15, 20, 30};

    int bagWeight = 4;  // 最大背包

    vector<vector<int>> dp(weight.size(), vector<int>(bagWeight + 1, 0));

    for (int j = weight[0]; j <= bagWeight; j++) {
        dp[0][j] = value[0];    // 第一行的初始化
    }

    for (int i = 1; i < weight.size(); i++) {   // 遍历物品
        for (int j = 1; j <= bagWeight; j++) {  // 遍历背包
            if (j < weight[i]) dp[i][j] = dp[i - 1][j]; // 放不了
            else dp[i][j] = max(dp[i - 1][j], dp[i - 1][j - weight[i]] + value[i]);
        }
    }
    return 0;
}
// 0 15 15 15 15 
// 0 15 15 20 35 
// 0 15 15 20 35 
```

#### 状态压缩

既然`dp[i][j]`只跟上一行有关，就可以只用一个滚动数组，但要注意，因为跟前面的元素有关，所以一定要先更新后面的元素，再更新前面的元素，**从后往前**

>+ 逆序遍历是为了保证每个物品只被放入一次
>+ 正序遍历会使得每个物品放入多次，变成完全背包

举例：物品0重量为1，价值为15
正序遍历
```cpp
dp[1] = dp[1 - weight[0]] + value[0] = 15
dp[2] = dp[2 - weight[0]] + value[0] = 30
```
这样物品0在背包2里面就放了两次

逆序遍历就不会出现这种状态的重叠
```cpp
dp[2] = dp[2 - weight[0]] + value[0] = 15   // dp[1]还没更新
dp[1] = dp[1 - weight[0]] + value[0] = 15
```

#### 代码

```cpp
int main(int argc, char const *argv[]) {
    vector<int> weight = {1, 3, 4};
    vector<int> value = {15, 20, 30};

    int bagWeight = 4;
    vector<int> dp(bagWeight + 1, 0);

    for (int i = 0; i < weight.size(); i++) {   // 遍历物品
        for (int j = bagWeight; j >= weight[i]; j--) {  // 从后往前遍历背包，直到放不进物品i
            dp[j] = max(dp[j], dp[j - weight[i]] + value[i]);
        }
    }
    return 0;
}
// 0 15 15 15 15 
// 0 15 15 20 35 
// 0 15 15 20 35 
```

