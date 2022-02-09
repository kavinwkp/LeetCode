## LeetCode 171. Excel Sheet Column Number

Given a string `columnTitle` that represents the column title as appear in an Excel sheet, return its corresponding column number.

For example:
```cpp
A -> 1
B -> 2
C -> 3
...
Z -> 26
AA -> 27
AB -> 28 
...
```

Example 1:

```cpp
Input: columnTitle = "AB"
Output: 28
```
Example 2:
```cpp
Input: columnTitle = "ZY"
Output: 701
```

### method

相当于26进制

```cpp
int titleToNumber(string columnTitle) {
    int res = 0;
    for (int i = 0; i < columnTitle.size(); i++) {
        int num = columnTitle[i] - 'A' + 1;
        res = res * 26 + num;
    }
    return res;
}
```