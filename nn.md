今天上了机器学习课，感觉啥也没听懂，遂怒看nn模块
# nn模块
（第一个想到的应用场景就是detector）
*阅读目标*
1. nn模块在整个视觉程序中的作用<——接收训练好的ONNX/其他格式的文件，部署到整个程序中
2. cuda、tensorrt等模块的作用
3. 机器学习相关知识
## yolo.h
起到了一个父类的作用
### 模块作用
1. 定义了一个结构体存放识别结果
2. 一个通用的、可扩展的 C++ 接口，用于实现基于 YOLOv8 架构的物体检测和关键点检测功能。
3. letterbox函数，实现缩放和填充。
4. GetObjects函数，处理神经网络推理后得到的原始数据 (output_data_)，并将其解码为可读的检测结果列表。
5. NMS函数，负责消除在同一物体上的重复、高度重叠的检测框。保留其中置信度最高的那个框
这三个函数都是虚函数，子类可继承生成自己的版本，通过基类访问派生类定义的函数）（具体实现见yolo.cpp
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
1. 通过 make_shared 帮助安全地管理 C++ 资源和可能由 TensorRT/CUDA 产生的资源。
2. 通过 checkRuntime 宏和函数，自动化并标准化了所有 CUDA API 调用的错误检查和报告。
3. 通过 TRTLogger 类，将 TensorRT 的内部消息集成到项目的通用日志系统 (glog) 中
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
## tensorrt
    题外话：cuda和tensorrt在机器学习过程中都充当了什么角色
    1. 关于tensorrt：高性能深度学习推理优化器。专注于推理阶段，最大限度地提高模型部署时的性能、吞吐量和效率。目标就是提升训练效率（大概）。SDK为开发者提供了优化和部署深度学习模型所需的一切工具和 API。
    2. 关于cuda：提供 C/C++ 编程接口，允许开发者编写能在 GPU 上并行运行的内核（Kernels），直接控制 GPU 内存和线程。
### 模块作用
1. 初始化与引擎构建 (Initialize, BuildEngineFromONNX, BuildEngineFromCache)
2. 运行推理 (Run)高效地在 GPU 上执行推理，并处理数据传输。
### cpp知识
1. override关键字：要求函数必须重写一个基类中同名的虚函数，避免常见的编程错误。
```cpp
  ~TensorRT() override;
  bool Initialize(std::string REF_IN model_file, int num_classes, int num_points) override;
  [[nodiscard]] std::vector<Objects> Run(cv::Mat image) override;
```
## yolo.cpp
1. 图像预处理：Yolo::LetterBox
该函数实现了 LetterBox 算法，目的是在不失真的情况下，将任意大小的图像缩放到网络固定的输入尺寸 (input_w_ x input_h_)。比例不够的需要填充（灰背景）

参数介绍：ro缩放比；dw&dh长和宽的填充量。（这三个参数会被保存，用于推理后将检测框坐标反向映射回原始图像。）
2. 网络输出解码：Yolo::GetObjects
该函数负责将 GPU 推理返回的原始一维浮点数数据 (output_data_) 转换为结构化的检测物体列表 (Objects)。遍历 YOLO 神经网络输出张量中的所有预测，并提取、解码出合格的边界框、类别和关键点信息。
3. 非极大值抑制：Yolo::NMS
该函数执行 非极大值抑制（NMS） 算法，用于去除对同一物体的高度重叠的冗余检测框，确保最终结果的精确性。