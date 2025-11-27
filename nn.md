今天上了机器学习课，感觉啥也没听懂，遂怒看nn模块
# nn模块
第一个想到的应用场景就是detector
*阅读目标*
1. 在本项目中使用了什么模型
2. cuda、tensorrt等模块的作用
3. 机器学习相关知识
## yolo.h
### 模块作用
1. 定义了一个结构体存放识别结果
2. 一个通用的、可扩展的 C++ 接口，用于实现基于 YOLOv8 架构的物体检测和关键点检测功能。
### cpp知识
1. 工厂模式
将对象的创建与使用分离。通过引入一个工厂函数来实现这个分离，这个工厂类负责管理和封装所有子类的构造细节。
```cpp
enable_factory(srm::nn, Yolo, CreateYolo);
```
2. 虚函数
根据传入的参数，调用对应的函数。
3. [[nodiscard]]
强调必须有返回值，否则报错。

## cuda-help
（coreml是对应apple操作系统的，忽略）
### 模块作用
核心目的是为 基于 TensorRT 和 CUDA 的深度学习推理代码提供一个健壮、可维护的基础
通过 make_shared 帮助安全地管理 C++ 资源和可能由 TensorRT/CUDA 产生的资源。
通过 checkRuntime 宏和函数，自动化并标准化了所有 CUDA API 调用的错误检查和报告。
通过 TRTLogger 类，将 TensorRT 的内部消息集成到项目的通用日志系统 (glog) 中
### cpp知识
1. 智能指针辅助函数
将原始指针 T* 转换为 std::shared_ptr<T>。
提供了一个统一的接口，可以扩展来管理任何资源（包括没有用智能指针创造的对象），保证他们能够正常析构
```cpp
template <typename T>
[[nodiscard]] std::shared_ptr<T> make_shared(T *ptr) {
  return std::shared_ptr<T>(ptr, [](T *p) { delete p; });
}
```
2. lambda函数
在上面的函数中，还用到了lambda函数：[](T *p) { delete p; }
主要优势是内联，不用再定义一个新的函数
$$\underbrace{[\text{捕获列表}]}_{\text{1. 外部变量}} \underbrace{(\text{参数列表})}_{\text{2. 输入参数}} \underbrace{\rightarrow \text{返回类型}}_{\text{3. (可选) 返回值}} \underbrace{\{\text{函数体}\}}_{\text{4. 执行代码}}$$
捕获列表 []： 它是空的。这表示这个 Lambda 函数不依赖于其定义范围内的任何局部变量。它是一个无状态的 Lambda。（一般会捕获局部变量）
参数列表 (T* p)： 当 std::shared_ptr 决定释放资源时，它会调用这个 Lambda，并把要释放的原始指针作为参数 p 传进去。
函数体 { delete p; }： 执行标准的内存释放
3. 宏定义
```cpp
#define checkRuntime(op) __check_cuda_runtime((op), #op, __FILE__, __LINE__)
[[nodiscard]] inline bool __check_cuda_runtime(cudaError_t code, char PTR_IN op, char PTR_IN file, int line) {
  if (code != cudaSuccess) {
    auto err_name = cudaGetErrorName(code);
    auto err_message = cudaGetErrorString(code);
    LOG(ERROR) << "runtime eror " << file << ": " << line << " " << op << " failed.\n code = " << err_name
               << ", message = " << err_message;
    return false;
  }
  return true;
}
```
这段代码定义了一个 C++ 宏 checkRuntime 和一个内联函数 __check_cuda_runtime
op是宏参数，想要检查的操作。checkRuntime(cudaMalloc(&ptr, size)) 时，op 就等于 cudaMalloc(&ptr, size)
(op) $\rightarrow$ cudaError_t code（状态码）：将 CUDA 函数的调用结果（即返回的 cudaError_t 类型的状态码）作为第一个参数传递给 __check_cuda_runtime 函数。
#op $\rightarrow$ char PTR_IN op（字符串）：#op 是 C/C++ 预处理器的一个字符串化（Stringification）操作符。它将传入的表达式 op（例如 cudaMalloc(&devPtr, size)）转换为一个字符串字面量。这使得错误日志能够清晰地打印出是哪个 CUDA 函数调用失败了。
__FILE__ $\rightarrow$ char PTR_IN file：这是一个标准预定义宏，提供当前源文件的文件名字符串。
__LINE__ $\rightarrow$ int line：这是一个标准预定义宏，提供宏被调用的行号整数。
4. inline
内联函数。编译器对一个被标记为 inline 的函数进行处理时，它可能会选择将函数体的内容直接插入（展开）到每个调用该函数的地方。（就是比如定义了一个inline int add（）函数，当在main中执行int c=add（1+3）时，直接替换成int c=1+3）节省开销
inline函数即使在不同源文件都有定义，也需要保持相同。