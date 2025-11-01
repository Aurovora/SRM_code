# Cppの特性

## coord模块

### auto关键字
```cpp
const auto ctv_iw_std = cfg.Get<std::vector<double>>({...});
```
自动推导变量类型，简化了代码。

### std::vector<T>
内存连续且大小可变。它主要用于存储可变长度的数据序列，或作为与外部接口(json)交互的数据格式。
```cpp
const auto ctv_iw_std = cfg.Get<std::vector<double>>({...});
```
std::vector<double>被用来承载配置中读取的xyz三个分量，然后将其拷贝到CTVec中

### 引用&
传递的只是原始对象所在内存地址的一个别名（在用法上是值，在本质上是地址的常量封装。修改引用也会修改变量），实现比指针更安全、更简洁的传址调用，避免昂贵的对象拷贝
+ 常量引用const&：保证数据的安全性和不变性

### Eigen库
~~线代C++~~
+ 表达式模板技术。
写下result = (A + B)*c 时，A + B不会立即执行计算或创建临时对象，而是创建了一个表示“A加B”的表达式对象。只有当结果被赋给result变量时，计算才会一次性执行，直接写入result的内存中，消除了中间对象的开销。
+ 链式赋值
类似operator<<重载，可快速、直观地初始化和赋值矩阵或向量的元素。
```cpp
rm_x << 1, 0, 0, 0, cos(ea[1]), -sin(ea[1]), 0, sin(ea[1]), cos(ea[1]);
```
