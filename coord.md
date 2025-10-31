# **coord模块**
## 模块概述
`coord`模块主要用于坐标系之间的转换和管理。它定义了多种常用的坐标表示方式（旋转矩阵、欧拉角、直角坐标、球坐标、齐次变换矩阵）及其相互转换的函数，并提供了一个`Solver`类来管理和执行在机器人或视觉系统中常见的几个关键坐标系（世界、IMU/陀螺仪、相机、枪口）之间的转换。

**base**: 实现单个坐标表示形式之间的基础数学转换

**solver**: 利用`base`提供的基础转换，结合系统配置参数，来管理和计算不同物理坐标系之间的复杂变换。
- - -
## base

### 欧拉角 $\leftrightarrow$ 旋转矩阵

#### 功能：

+ **EAngleToRMat**： 将一个3×3的矩阵rm转换为特定的欧拉角(yaw,pitch,row)序列。

+ **EAngleToRMat**： 将一个欧拉角ea(ea[0],ea[1],ea[2])(yaw,pitch,row)转换为3×3的矩阵

绕X轴旋转矩阵 $\mathbf{R}_x$ (Pitch)：

$$
\mathbf{R}_x(\theta) = \begin{pmatrix} 
1 & 0 & 0 \\
0 & \cos\theta & -\sin\theta \\ 
0 & \sin\theta & \cos\theta 
\end{pmatrix}
$$

绕 $Y$ 轴旋转矩阵 $\mathbf{R}_y$ (Yaw)：

$$
\mathbf{R}_y(\phi) = \begin{pmatrix} 
\cos\phi & 0 & \sin\phi \\
0 & 1 & 0 \\ 
-\sin\phi & 0 & \cos\phi 
\end{pmatrix}
$$

绕 $Z$ 轴旋转矩阵 $\mathbf{R}_z$ (Roll)：

$$
\mathbf{R}_z(\psi) = \begin{pmatrix} 
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1 \end{pmatrix}
$$

最终旋转矩阵R是这三个基本旋转矩阵的乘积(乘法顺序是rm_y * rm_x * rm_z)

#### 代码：
```cpp
EAngle RMatToEAngle(RMat REF_IN rm) {
  if (rm(1, 0) * rm(1, 0) + rm(1, 1) * rm(1, 1) < 1e-8) {
    return {atan2(rm(0, 1), rm(2, 1)), asin(-rm(1, 2)), 0};
  } else {
    return {atan2(rm(0, 2), rm(2, 2)), asin(-rm(1, 2)), atan2(rm(1, 0), rm(1, 1))};
  }
}
RMat EAngleToRMat(EAngle REF_IN ea) {
  RMat rm_x, rm_y, rm_z;
  rm_x << 1, 0, 0, 0, cos(ea[1]), -sin(ea[1]), 0, sin(ea[1]), cos(ea[1]);
  rm_y << cos(ea[0]), 0, sin(ea[0]), 0, 1, 0, -sin(ea[0]), 0, cos(ea[0]);
  rm_z << cos(ea[2]), -sin(ea[2]), 0, sin(ea[2]), cos(ea[2]), 0, 0, 0, 1;
  return rm_y * rm_x * rm_z;
}
```

### 直角坐标 $\leftrightarrow$ 球坐标
#### 功能：
+ **STVecToCTVec**: 球坐标(phi, theta, r)右偏角，上仰角，转换到直角坐标(x,y,z)

+ **CTVecToSTVec**: 直角坐标转换到球坐标

$$x = r\times\cos(theta) \times \sin(phi)$$

$$y = r \times(-\sin(theta))$$

$$z = r \times\cos(theta) \times \cos(phi)$$

#### 代码：
```cpp
CTVec STVecToCTVec(STVec REF_IN stv) {
  return {stv.z() * cos(stv.y()) * sin(stv.x()), -stv.z() * sin(stv.y()), stv.z() * cos(stv.y()) * cos(stv.x())};
}

STVec CTVecToSTVec(CTVec REF_IN ctv) {
  return {atan2(ctv.x(), ctv.z()), atan2(-ctv.y(), sqrt(ctv.x() * ctv.x() + ctv.z() * ctv.z())),
          sqrt(ctv.x() * ctv.x() + ctv.y() * ctv.y() + ctv.z() * ctv.z())};
}
```

### 旋转向量 $\leftrightarrow$ 旋转矩阵

#### 功能：(借助opencv的Rodrigues函数，因此在计算过程中需要进行格式转换)

+ **RMatToRVec**：将3×1的旋转向量rv转换为3×3的旋转矩阵rm。

+ **RVecToRMat**：将一个3×3的旋转矩阵rm转换为一个3×1的旋转向量。

#### 代码：
```cpp
RMat RVecToRMat(RVec REF_IN rv) {
  cv::Mat rv_cv, rm_cv;
  RMat rm;
  eigen2cv(rv, rv_cv);
  Rodrigues(rv_cv, rm_cv);
  cv2eigen(rm_cv, rm);
  return rm;
}
RVec RMatToRVec(RMat REF_IN rm) {
  cv::Mat rv_cv, rm_cv;
  RVec rv;
  eigen2cv(rm, rm_cv);
  Rodrigues(rm_cv, rv_cv);
  cv2eigen(rv_cv, rv);
  return rv;
}
```
- - -
## solver

### 初始化与配置
#### 功能：
+ **Initialize**： 从config配置中读取所有坐标系（IMU、相机、枪口）相对于世界系或IMU的固定位姿（平移ctv和旋转ea），并将角度转换为弧度、平移量转换为米。(IMU是一种传感器模块，用于测量物体在空间中的姿态、角速度和加速度。)

### Camera $\leftrightarrow$ World
#### 功能(涵盖了相机坐标、IMU系、世界系的互转)
    * 世界坐标系：原点为陀螺仪中心，自身无旋转，方向为陀螺仪置零的方向
    * 陀螺仪IMU坐标系：原点为陀螺仪中心，自身可旋转
    * 相机坐标系：原点为相机光心，自身可旋转，与世界坐标系原点位置关系固定
    * 枪口坐标系：原点为子弹获得初速的位置，自身可旋转，与世界坐标系原点位置关系固定
+ **CamToWorld**：将相机坐标系坐标转换为世界坐标系坐标。（需借助IMU坐标）

$$\text(相机坐标ctv_cam)\xrightarrow\text(转换到IMU坐标系)\xrightarrow\text(添加IMU固定平移偏移ctv_iw_)\xrightarrow\text(乘上实时IMU姿态rm_imu)\xrightarrow\text(世界坐标)$$

+ **WorldToCam**：将世界坐标系坐标转换为相机坐标系坐标

```cpp
CTVec Solver::CamToWorld(CTVec REF_IN ctv_cam, RMat REF_IN rm_imu) const {
  HCTVec hctv_cam;
  CTVec ctv_imu;
  hctv_cam << ctv_cam[0], ctv_cam[1], ctv_cam[2], 1;
  HCTVec hctv_imu = htm_ci_ * hctv_cam;//将相机坐标系下的点，通过固定的齐次变换矩阵转换到IMU坐标系下。
  ctv_imu << hctv_imu[0], hctv_imu[1], hctv_imu[2];
  ctv_imu += ctv_iw_;//添加IMU本身在世界坐标系中有的固定平移偏移。
  return rm_imu * ctv_imu;//乘上IMU的实时姿态旋转矩阵。这使点从IMU坐标系旋转到世界坐标系，实现了动态姿态的转换。
}
CTVec Solver::WorldToCam(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const {
  HCTVec hctv_imu;
  CTVec ctv_imu = rm_imu.transpose() * ctv_world;
  ctv_imu -= ctv_iw_;
  hctv_imu << ctv_imu[0], ctv_imu[1], ctv_imu[2], 1;
  HCTVec hctv_cam = htm_ic_ * hctv_imu;
  return {hctv_cam[0], hctv_cam[1], hctv_cam[2]};
}
```

### 枪口坐标系 $\leftrightarrow$ 世界坐标系转换
#### 功能
+ **MuzzleToWorld**: 将枪口坐标系坐标转换为世界坐标系坐标
+ **WorldToMuzzle**: 将世界坐标系坐标转换为枪口坐标系坐标

```cpp
CTVec Solver::MuzzleToWorld(CTVec REF_IN ctv_muzzle, RMat REF_IN rm_imu) const {
  HCTVec hctv_muzzle;
  CTVec ctv_imu;
  hctv_muzzle << ctv_muzzle[0], ctv_muzzle[1], ctv_muzzle[2], 1;
  HCTVec hctv_imu = htm_mi_ * hctv_muzzle;
  ctv_imu << hctv_imu[0], hctv_imu[1], hctv_imu[2];
  ctv_imu += ctv_iw_;
  return rm_imu * ctv_imu;
}

CTVec Solver::WorldToMuzzle(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const {
  HCTVec hctv_imu;
  CTVec ctv_imu = rm_imu.transpose() * ctv_world;
  ctv_imu -= ctv_iw_;
  hctv_imu << ctv_imu[0], ctv_imu[1], ctv_imu[2], 1;
  HCTVec hctv_muzzle = htm_im_ * hctv_imu;
  return {hctv_muzzle[0], hctv_muzzle[1], hctv_muzzle[2]};
}
```

### 相机坐标系下的三维点到二维图像像素坐标的投影
## 功能
+ **CamToPic**: 将相机坐标系下的三维点ctv_cam投影到二维图像平面上的像素坐标cv::Point2f

```cpp
cv::Point2f Solver::CamToPic(CTVec REF_IN ctv_cam) const {
  CTVec result = (1.f / ctv_cam.z()) * intrinsic_mat_eigen_ * ctv_cam;//将相机坐标与内参矩阵相乘。这一步将点从相机坐标系转换到图像像素尺度,并除以z轴分量，将点从三维空间压扁到二维图像平面
  return {static_cast<float>(result.x()), static_cast<float>(result.y())};
}
```
此模块的代码中使用了多个现代 C++ 特性，它们提高了代码的清晰度、安全性和效率。

1. 命名空间 (namespace)
代码使用了嵌套命名空间 namespace srm::coord { ... } (C++17 简化写法)，这有助于组织代码，避免全局命名冲突，并清晰地标识模块归属。

2. 属性标记 ([[nodiscard]]) - C++17
在所有转换函数（如 RMatToEAngle、CamToWorld）和 Initialize() 方法前都使用了 [[nodiscard]] 属性。

含义： 标记函数的返回值不应被忽略。

好处： 增强代码安全性。如果调用者没有使用函数的返回值（例如没有检查 Initialize() 的返回值），编译器会发出警告，提醒开发者可能遗漏了重要的状态信息或计算结果。

3. 类型别名 (using) - C++11
模块使用 using 关键字为复杂的 Eigen 类型创建了别名，例如：

C++

using RVec = Eigen::Vector3d;
using RMat = Eigen::Matrix3d;
// ...
好处： 提高了代码的可读性，使代码更具领域特定性（一眼看出 RMat 是旋转矩阵，而不是普通的 Eigen::Matrix3d），且比传统的 typedef 更灵活。

4. constexpr (隐含应用，与库相关) - C++11/14
虽然代码中没有直接使用 constexpr，但代码高度依赖 Eigen 库。现代 C++ 库（如 Eigen 和 STL）通常会在容器、数学函数等处使用 constexpr 来允许在编译期进行计算，这对于性能敏感的坐标转换非常重要，可以减少运行时开销。

5. 增强的枚举（未显式使用，但常用于类似场景） - C++11
在定义坐标系类型时，如果使用 enum class（作用域内枚举）而不是传统的 enum，可以进一步避免命名空间污染和类型转换问题，是现代 C++ 的最佳实践。虽然此模块主要使用类型别名，但这是相关领域的常见特性。

6. 统一初始化 ({}) - C++11
在 Solver::Initialize() 中，从配置读取 std::vector<double> 后，使用 {} 语法将数据安全地初始化到 Eigen 向量中，例如：

C++

// ctv_iw_ 是 Eigen::Vector3d
ctv_iw_ << ctv_iw_std[0], ctv_iw_std[1], ctv_iw_std[2]; // 使用 Eigen 的逗号初始化（stream-like）

// 如果是直接使用列表，统一初始化更好
// CTVec ctv_example{1.0, 2.0, 3.0};
代码主要使用了 Eigen 的流式初始化，但在现代 C++ 中，{} 统一初始化是推荐用于防止隐式类型转换和保证零初始化的方式。

cpp特性：
引用&：传递的只是原始对象所在内存地址的一个别名（在用法上是值，在本质上是地址的常量封装。修改引用也会修改变量），实现比指针更安全、更简洁的传址调用，避免昂贵的对象拷贝
常量引用const&：保证数据的安全性和不变性