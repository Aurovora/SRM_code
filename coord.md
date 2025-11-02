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

#### 功能：(借助opencv的Rodrigues函数（用于实现两种旋转表示方法之间的相互转换），因此在计算过程中需要进行格式转换)

+ **RVecToRMat**：将3×1的旋转向量rv转换为3×3的旋转矩阵rm。

+ **RMatToRVec**：将一个3×3的旋转矩阵rm转换为一个3×1的旋转向量。

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
    CTVec ctv_iw_;  ///< 陀螺仪相对世界坐标系原点的位移
    CTVec ctv_ci_;  ///< 相机相对陀螺仪的位移
    CTVec ctv_mi_;  ///< 枪口相对陀螺仪的位移
    CTVec ctv_cw_;  ///< 相机相对世界坐标系原点的位移
    CTVec ctv_mw_;  ///< 枪口相对世界坐标系原点的位移
    CTVec ctv_mc_;  ///< 枪口相对相机的位移
    RMat rm_ci_;    ///< 相机相对陀螺仪的旋转矩阵
    RMat rm_mi_;    ///< 枪口相对陀螺仪的旋转矩阵
    HTMat htm_ic_;  ///< 陀螺仪坐标系转换到相机坐标系
    HTMat htm_ci_;  ///< 相机坐标系转换到陀螺仪坐标系
    HTMat htm_im_;  ///< 陀螺仪坐标系转换到枪口坐标系
    HTMat htm_mi_;  ///< 枪口坐标系转换到陀螺仪坐标系
### 初始化与配置
#### 功能：
+ **Initialize**： 初始化
1. 从config配置中读取所有位移向量
```cpp
auto prefix = "coord." + cfg.Get<std::string>({"type"});
  const auto ctv_iw_std = cfg.Get<std::vector<double>>({prefix, "ctv_imu_world"});
  const auto ctv_ci_std = cfg.Get<std::vector<double>>({prefix, "ctv_cam_imu"});
  const auto ctv_mi_std = cfg.Get<std::vector<double>>({prefix, "ctv_muzzle_imu"});
  const auto ea_ci_std = cfg.Get<std::vector<double>>({prefix, "ea_cam_imu"});
  const auto ea_mi_std = cfg.Get<std::vector<double>>({prefix, "ea_muzzle_imu"});
```
2. 将角度转换为弧度、平移量转换为米。

3. 齐次变换矩阵预计算(解决三维空间中旋转和平移操作不能统一表示的问题)
```cpp
//构造相机坐标系到 IMU 坐标系的齐次变换矩阵
  htm_ci_ << rm_ci_(0, 0), rm_ci_(0, 1), rm_ci_(0, 2), ctv_ci_.x(), rm_ci_(1, 0), rm_ci_(1, 1), rm_ci_(1, 2),
  ctv_ci_.y(), rm_ci_(2, 0), rm_ci_(2, 1), rm_ci_(2, 2), ctv_ci_.z(), 0, 0, 0, 1;
//构造枪口坐标系到 IMU 坐标系的齐次变换矩阵
  htm_mi_ << rm_mi_(0, 0), rm_mi_(0, 1), rm_mi_(0, 2), ctv_mi_.x(), rm_mi_(1, 0), rm_mi_(1, 1), rm_mi_(1, 2),
      ctv_mi_.y(), rm_mi_(2, 0), rm_mi_(2, 1), rm_mi_(2, 2), ctv_mi_.z(), 0, 0, 0, 1;
//初始化相机内参（从 OpenCV 格式转换为 Eigen 格式）
  cv2eigen(intrinsic_mat_, intrinsic_mat_eigen_);
  htm_ic_ = htm_ci_.inverse();
  htm_im_ = htm_mi_.inverse();
```

### Camera $\leftrightarrow$ World
#### 功能(涵盖了相机坐标、IMU系、世界系的互转)
    * 世界坐标系：原点为陀螺仪中心，自身无旋转，方向为陀螺仪置零的方向
    * 陀螺仪IMU坐标系：原点为陀螺仪中心，方向随云台/机器人姿态实时变化
    * 相机坐标系：原点为相机光心，自身可旋转，与世界坐标系原点位置关系固定
    * 枪口坐标系：原点为子弹获得初速的位置，自身可旋转，与世界坐标系原点位置关系固定
+ **CamToWorld**：将相机坐标系坐标转换为世界坐标系坐标。（需借助IMU坐标）

$$相机坐标ctv_cam\rightarrow转换到IMU坐标系\rightarrow添加IMU固定平移偏移ctv_iw_\rightarrow乘上实时IMU姿态rm_imu\rightarrow世界坐标$$

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

### 三维点 $\rightarrow$ 二维图像像素坐标
#### 功能
+ **CamToPic**: 将相机坐标系下的三维点ctv_cam投影到二维图像平面上的像素坐标cv::Point2f

```cpp
cv::Point2f Solver::CamToPic(CTVec REF_IN ctv_cam) const {
  CTVec result = (1.f / ctv_cam.z()) * intrinsic_mat_eigen_ * ctv_cam;//将相机坐标与内参矩阵相乘。这一步将点从相机坐标系转换到图像像素尺度,并除以z轴分量，将点从三维空间压扁到二维图像平面
  return {static_cast<float>(result.x()), static_cast<float>(result.y())};
}
```
