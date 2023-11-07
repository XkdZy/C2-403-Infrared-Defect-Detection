# C2-403-Infrared-Defect-Detection

## 0.环境依赖
    实验平台：  
      * Windows10+VS2019  
    第三方库：  
      * opecv4.5.5  
      * open3dv0.10.0  
      * AstraSDK（RGBD深度相机）  
      * rokaeSDK（珞石机械臂） 
      * Anaconda VILD环境（目标检测模型）
  
## 1.环境配置
### 包含目录
        项目右击->属性->配置属性->VC++目录->包含目录  
#### rokae
        lib_root/rokae/include  
#### open3d
        lib_root/Open3D_v0.10.0/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/Eigen  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/GLFW/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/glew/include  
        lib_root/Open3D_v0.10.0/include/Open3D/3rdparty/fmt/include  
### 库目录
        项目右击->属性->配置属性->VC++目录->库目录
### 链接器输入
        项目右击->属性->配置属性->链接器->输入
