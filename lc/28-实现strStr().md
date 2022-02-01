## LeetCode 28. Implement strStr()

Return the **index** of the first occurrence of `needle` in `haystack`, or `-1` if `needle` is not part of `haystack`.

Clarification:

What should we return when needle is an empty string? This is a great question to ask during an interview.

Example 1:
```cpp
Input: haystack = "hello", needle = "ll"
Output: 2
```

### method: KMP

文本串：`aabaabaaf`
模式串：`aabaaf`

`next`数组记录最大的前后缀相等的长度，这样当某个元素不匹配的时候，就可以跳到最大相等前缀的位置重新匹配，而不用跳到开头

因为知道了这一串的某个后缀与前缀是相等的，所以就直接跳到相等前缀的位置开始匹配，节省时间

1、求next数组

+ 定义两个指针`i`和`j`，`j`指向前缀末尾位置，还表示`i`及`i`之前的子串的最长相等前后缀长度，初始化为0
+ `i`指向后缀末尾位置，从1到末尾遍历文本串
+ `s[j]!=s[i]`时，`j`要根据前一个字符的`next`值回退，直至退到`0`
+ `s[j]==s[i]`时，`j`递增
+ `i`位置的`next`值就是`j`

<center>
<img src=https://github.com/kavinwkp/blogimage/raw/main/img/LeetCode/28/28.png>
</center>

2、文本串和模式串匹配

```cpp
模式串: a a b a a f
next : 0 1 0 1 2 0
```

第一次到`f`时不匹配

```cpp
文本串: a a b a a b a a f
模式串: a a b a a f
next : 0 1 0 1 2 0
```

`f`前面的`a`的`next`值为`2`，所以跳到`2`的位置重新匹配

```cpp
文本串: a a b a a b a a f
模式串:       a a b a a f
next :       0 1 0 1 2 0
```

这样就匹配上了

```cpp
vector<int> next;
void getNext(string s) {
    int j = 0;
    for (int i = 1; i < s.size(); i++) {
        while (j > 0 && s[j] != s[i]) { // 要用while
            j = next[j - 1];    // j要回退
        }
        if (s[j] == s[i]) j++;
        next[i] = j;
    }
}
int strStr(string haystack, string needle) {
    if (needle.size() == 0) return 0;
    next.resize(needle.size(), 0);
    getNext(needle);
    int j = 0;
    for (int i = 0; i < haystack.size(); i++) { // 遍历文本串
        while (j > 0 && needle[j] != haystack[i]) {
            j = next[j - 1];
        }
        if (needle[j] == haystack[i]) j++;
        if (j == needle.size()) return i - j + 1;   // j刚好是模式串的长度
    }
    return -1;
}
```

时间复杂度：$O(m + n)$
空间复杂度：$O(n)$